#pragma once
#include <opencv2/core.hpp>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>

namespace memory_diagnostics {
// Feature owners update this ledger on the serial VIO thread. The key does not
// retain the allocation. Count the allocation once, even for Mat views/ROIs.
struct ImageRecord { std::uint64_t refs = 0; std::size_t bytes = 0; };
inline std::unordered_map<const cv::UMatData*, ImageRecord> images;
inline std::uint64_t points_created = 0, points_deleted = 0;
inline std::uint64_t features_created = 0, features_deleted = 0;
inline std::uint64_t patches_allocated = 0, skipped_patches = 0;
inline std::uint64_t image_bytes = 0, image_refs = 0, external_images = 0;
inline std::uint64_t image_last_ref_freed = 0, image_detached_external_ref = 0;
inline std::uint64_t bytes_last_ref_freed = 0, bytes_detached_external_ref = 0;
inline std::uint64_t images_created = 0, images_released = 0, bytes_created = 0, bytes_released = 0;
inline void retain(const cv::Mat& image) {
  if (!image.u) { if (!image.empty()) ++external_images; return; }
  auto& record = images[image.u];
  if (record.refs++ == 0) {
    record.bytes = image.u->size;
    image_bytes += record.bytes;
    bytes_created += record.bytes;
    ++images_created;
  }
  ++image_refs;
}
inline void release(const cv::Mat& image) {
  if (!image.u) { if (!image.empty()) --external_images; return; }
  auto it = images.find(image.u);
  if (it == images.end()) std::terminate();
  --image_refs;
  if (--it->second.refs) return;
  const auto bytes = it->second.bytes;
  if (image.u->refcount == 1 && image.u->urefcount == 0 && !(image.u->flags & cv::UMatData::USER_ALLOCATED)) {
    // The Feature's Mat destructor follows immediately; tested with an allocator callback.
    ++image_last_ref_freed;
    bytes_last_ref_freed += bytes;
  } else {
    ++image_detached_external_ref;
    bytes_detached_external_ref += bytes;
  }
  image_bytes -= bytes;
  bytes_released += bytes;
  ++images_released;
  images.erase(it);
}

struct Resources {
  std::uint64_t total = 0, available = 0, rss = 0, disk_free = 0;
  bool memory_valid = false;
  std::uint64_t shortfall() const {
    const auto reserve = total / 8; // host headroom; independent of image count/bytes
    return memory_valid && available < reserve ? reserve - available : 0;
  }
};

inline Resources readResources() {
  Resources result;
  std::ifstream meminfo("/proc/meminfo");
  std::string key, unit;
  std::uint64_t kib;
  bool have_total = false, have_available = false;
  while (meminfo >> key >> kib >> unit) {
    if (key == "MemTotal:") { result.total = kib * 1024; have_total = true; }
    if (key == "MemAvailable:") { result.available = kib * 1024; have_available = true; }
    if (have_total && have_available) break;
  }
  result.memory_valid = have_total && have_available && result.total > 0;
  std::ifstream status("/proc/self/status");
  std::string line;
  while (std::getline(status, line)) {
    if (line.compare(0, 6, "VmRSS:") == 0) {
      std::istringstream field(line.substr(6));
      if (field >> kib) result.rss = kib * 1024;
      break;
    }
  }
  std::error_code ec;
  const auto space = std::filesystem::space(".", ec);
  if (!ec) result.disk_free = space.available;
  return result;
}

} // namespace memory_diagnostics
