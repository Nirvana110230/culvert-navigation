#pragma once
#include <cmath>
#include "voxel_map.h"

// Visual cells are [k * 0.5, (k + 1) * 0.5) on each world-coordinate axis.
// Input must be finite and its index representable by int64_t, as for VOXEL_LOCATION.
inline VOXEL_LOCATION visualVoxelKey(const V3D& point)
{
  constexpr double voxel_size = 0.5;
  return VOXEL_LOCATION(static_cast<int64_t>(std::floor(point[0] / voxel_size)),
                        static_cast<int64_t>(std::floor(point[1] / voxel_size)),
                        static_cast<int64_t>(std::floor(point[2] / voxel_size)));
}
