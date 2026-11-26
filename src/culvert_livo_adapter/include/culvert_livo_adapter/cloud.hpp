#pragma once

#include "culvert_livo_adapter/kinematics.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <algorithm>
#include <array>
#include <cstring>
#include <limits>
#include <string>

namespace culvert
{
using Cloud = sensor_msgs::msg::PointCloud2;
using Field = sensor_msgs::msg::PointField;
constexpr size_t max_cloud_bytes = 8 * 1024 * 1024;

// Explicit XY(Z,I) output prevents stale normal vectors or relative times surviving a frame change.
inline bool transform_cloud(const Cloud &src, const Transform &base_from_lidar, Cloud &dst,
                            std::string &error)
{
  const uint64_t n = uint64_t(src.width) * src.height;
  if (n == 0 || n > 300000 || src.is_bigendian || src.point_step == 0 ||
      uint64_t(src.width) * src.point_step > src.row_step ||
      uint64_t(src.height) * src.row_step != src.data.size() || src.data.size() > max_cloud_bytes) {
    error = "unsupported_cloud_layout_or_size";
    return false;
  }
  std::array<uint32_t, 4> offset{};
  std::array<bool, 4> found{};
  const std::array<std::string, 4> names{"x", "y", "z", "intensity"};
  for (const auto &field : src.fields) {
    auto it = std::find(names.begin(), names.end(), field.name);
    if (it == names.end()) continue;
    const auto i = static_cast<size_t>(std::distance(names.begin(), it));
    if (found[i] || field.count != 1 || field.datatype != Field::FLOAT32 ||
        uint64_t(field.offset) + sizeof(float) > src.point_step) {
      error = "invalid_cloud_field";
      return false;
    }
    found[i] = true;
    offset[i] = field.offset;
  }
  if (!found[0] || !found[1] || !found[2]) {
    error = "missing_xyz";
    return false;
  }
  const size_t channels = found[3] ? 4 : 3;
  dst.header = src.header;
  dst.height = src.height;
  dst.width = src.width;
  dst.fields.resize(channels);
  for (size_t i = 0; i < channels; ++i) {
    dst.fields[i].name = names[i];
    dst.fields[i].count = 1;
    dst.fields[i].datatype = Field::FLOAT32;
    dst.fields[i].offset = i * sizeof(float);
  }
  dst.point_step = channels * sizeof(float);
  dst.row_step = dst.width * dst.point_step;
  dst.is_bigendian = false;
  dst.is_dense = true;
  dst.data.resize(n * dst.point_step);
  for (size_t row = 0; row < src.height; ++row) {
    for (size_t col = 0; col < src.width; ++col) {
      const auto *point = src.data.data() + row * src.row_step + col * src.point_step;
      std::array<float, 4> values{};
      for (size_t i = 0; i < channels; ++i) {
        std::memcpy(&values[i], point + offset[i], sizeof(float));
        if (!std::isfinite(values[i])) {
          error = "nonfinite_cloud";
          return false;
        }
      }
      const Vector transformed = base_from_lidar * Vector(values[0], values[1], values[2]);
      for (size_t i = 0; i < 3; ++i) {
        if (!std::isfinite(transformed[i]) ||
            std::abs(transformed[i]) > std::numeric_limits<float>::max()) {
          error = "nonfinite_transformed_cloud";
          return false;
        }
        values[i] = static_cast<float>(transformed[i]);
      }
      std::memcpy(dst.data.data() + row * dst.row_step + col * dst.point_step, values.data(),
                  dst.point_step);
    }
  }
  return true;
}
}  // namespace culvert
