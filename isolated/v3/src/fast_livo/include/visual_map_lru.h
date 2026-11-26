#pragma once
#include "feature.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <unordered_set>
#include <vector>

// Called by the serial VIO owner. Only visual cells own these points. LiDAR cells
// and the accumulated cloud/trajectory archive have independent lifetimes.
namespace visual_lru {
struct Options {
  bool enabled = true;
  double radius_m = 30.0;
  std::uint64_t far_idle_frames = 30, idle_frames = 100, recent_frames = 10;

  void validate() const {
    if (!std::isfinite(radius_m) || radius_m <= 0 || !recent_frames ||
        far_idle_frames < recent_frames || idle_frames < far_idle_frames)
      throw std::invalid_argument("invalid visual local map radius or age window");
  }
};

struct Result {
  std::uint64_t bytes_before = 0, bytes_after = 0, freed_bytes = 0;
  std::uint64_t points_evicted = 0, features_evicted = 0;
  std::uint64_t far_evicted = 0, idle_evicted = 0, pressure_points_evicted = 0;
  std::uint64_t observations_pruned = 0, borrowed_points = 0, blocked_points = 0;
  std::uint64_t active_images = 0, protected_images = 0, protected_bytes = 0;
};

inline std::uint64_t age(std::uint64_t now, std::uint64_t then) {
  return now >= then ? now - then : 0;
}

// In normal operation keep enough views for the upstream >10-view convergence
// check. Under actual memory pressure keep at least six for normal/reference
// estimation. Always keep the selected reference, newest view and recent uses.
inline void removableHistory(const VisualPoint& point, std::uint64_t frame,
                             std::uint64_t keep_age, std::size_t keep_views,
                             std::vector<Feature*>& out) {
  out.clear();
  if (point.obs_.size() <= keep_views) return;
  for (auto* feature : point.obs_) {
    if (feature != point.ref_patch && feature != point.obs_.front() &&
        age(frame, feature->last_used_frame_) >= keep_age)
      out.push_back(feature);
  }
  std::sort(out.begin(), out.end(), [](const Feature* a, const Feature* b) {
    if (a->last_used_frame_ != b->last_used_frame_) return a->last_used_frame_ < b->last_used_frame_;
    return a->lru_id_ < b->lru_id_;
  });
  const auto removable = point.obs_.size() - keep_views;
  if (out.size() > removable) out.resize(removable);
}

struct Candidate {
  VisualPoint** slot;
  bool far;
};

template<class Map>
Result maintain(Map& map, const Eigen::Vector3d& center, std::uint64_t frame,
                const std::unordered_set<VisualPoint*>& borrowed, const Options& options,
                std::uint64_t system_shortfall = 0, bool sample = false) {
  options.validate();
  Result result;
  result.bytes_before = result.bytes_after = memory_diagnostics::image_bytes;
  if (!options.enabled) return result;
  if (!center.allFinite()) throw std::runtime_error("nonfinite local visual map center");

  const auto freed_before = memory_diagnostics::bytes_last_ref_freed;
  auto needs_relief = [&] {
    return memory_diagnostics::bytes_last_ref_freed - freed_before < system_shortfall;
  };
  auto remove_point = [&](VisualPoint*& point) {
    result.features_evicted += point->obs_.size();
    ++result.points_evicted;
    delete point; // Feature owns the patch and Mat header; OpenCV owns the backing buffer.
    point = nullptr;
  };

  std::vector<Feature*> redundant;
  std::vector<Candidate> inactive;
  for (auto& cell : map) for (auto& point : cell.second->voxel_points) {
    if (borrowed.count(point)) continue; // A live external consumer pins the whole point.
    const auto unused = age(frame, point->last_used_frame_);
    const bool recent = age(frame, point->created_frame_) < options.recent_frames ||
                        unused < options.recent_frames;
    const bool far = (point->pos_ - center).squaredNorm() > options.radius_m * options.radius_m;
    if (!recent && ((far && unused >= options.far_idle_frames) || unused >= options.idle_frames)) {
      if (far) ++result.far_evicted; else ++result.idle_evicted;
      remove_point(point);
      continue;
    }

    // A recently matched point does not pin every historical observation.
    const bool pressured = needs_relief();
    removableHistory(*point, frame, pressured ? options.recent_frames : options.idle_frames,
                     pressured ? 6 : 11, redundant);
    for (auto* feature : redundant) {
      if (pressured && !needs_relief()) break;
      point->deleteFeatureRef(feature);
      ++result.observations_pruned;
      ++result.features_evicted;
    }
    if (!recent && needs_relief()) inactive.push_back({&point, far});
  }

  if (needs_relief()) {
    std::sort(inactive.begin(), inactive.end(), [](const Candidate& a, const Candidate& b) {
      if (a.far != b.far) return a.far > b.far;
      const auto* p = *a.slot;
      const auto* q = *b.slot;
      if (p->last_used_frame_ != q->last_used_frame_) return p->last_used_frame_ < q->last_used_frame_;
      return p->lru_id_ < q->lru_id_;
    });
    for (const auto& candidate : inactive) {
      if (!needs_relief()) break;
      remove_point(*candidate.slot);
      ++result.pressure_points_evicted;
    }
  }

  // Slots remain stable until all LRU candidates have been handled. Compact once.
  std::unordered_set<const cv::UMatData*> active_images, protected_images;
  for (auto cell = map.begin(); cell != map.end();) {
    auto& points = cell->second->voxel_points;
    if (result.points_evicted) points.erase(std::remove(points.begin(), points.end(), nullptr), points.end());
    cell->second->count = static_cast<int>(points.size());
    if (points.empty()) {
      delete cell->second;
      cell = map.erase(cell);
      continue;
    }
    if (sample) for (auto* point : points) {
      const bool pinned = borrowed.count(point);
      const bool recent = age(frame, point->created_frame_) < options.recent_frames ||
                          age(frame, point->last_used_frame_) < options.recent_frames;
      if (pinned) ++result.borrowed_points;
      if (pinned || recent) {
        ++result.blocked_points;
        removableHistory(*point, frame, options.recent_frames, 6, redundant);
        for (const auto* feature : point->obs_) {
          auto* allocation = feature->img_.u;
          if (!allocation) continue;
          if (pinned || feature->last_used_frame_ == frame) active_images.insert(allocation);
          if (pinned || std::find(redundant.begin(), redundant.end(), feature) == redundant.end())
            protected_images.insert(allocation);
        }
      }
    }
    ++cell;
  }
  result.active_images = active_images.size();
  result.protected_images = protected_images.size();
  for (auto* image : protected_images) result.protected_bytes += image->size;
  result.freed_bytes = memory_diagnostics::bytes_last_ref_freed - freed_before;
  result.bytes_after = memory_diagnostics::image_bytes;
  // If live users still pin memory, return normally. The next frame can use
  // those observations and the next system sample can request more relief.
  return result;
}
} // namespace visual_lru
