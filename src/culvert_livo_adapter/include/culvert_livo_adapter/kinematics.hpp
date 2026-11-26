#pragma once

#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

namespace culvert
{
// T_A_B maps a point in B to A. Distances use metres; quaternions are xyzw on the wire.
using Transform = Eigen::Isometry3d;
using Vector = Eigen::Vector3d;
using Quaternion = Eigen::Quaterniond;
constexpr double standard_gravity = 9.80665;

inline std::optional<Transform> transform(const Vector &translation, Quaternion q)
{
  const double norm = q.coeffs().stableNorm();
  if (!translation.allFinite() || !std::isfinite(norm) || std::abs(norm - 1.0) > 0.05) {
    return {};
  }
  q.coeffs() /= norm;
  Transform result = Transform::Identity();
  result.linear() = q.toRotationMatrix();
  result.translation() = translation;
  return result;
}

inline std::optional<Transform> extrinsic(const std::vector<double> &t,
                                          const std::vector<double> &xyzw)
{
  if (t.size() != 3 || xyzw.size() != 4) {
    return {};
  }
  return transform(Vector(t[0], t[1], t[2]), Quaternion(xyzw[3], xyzw[0], xyzw[1], xyzw[2]));
}

struct Twist
{
  Vector linear;
  Vector angular;
};

inline bool exceeds_motion_limit(const Twist &twist, double linear_mps, double angular_radps)
{
  return twist.linear.stableNorm() > linear_mps || twist.angular.stableNorm() > angular_radps;
}

inline std::optional<Twist> difference(const Transform &prev_base, const Transform &base,
                                       double dt, double min_dt, double max_dt)
{
  if (!std::isfinite(dt) || dt < min_dt || dt > max_dt || dt <= 0) {
    return {};
  }
  // The logarithm takes the shortest rotation. Its axis is shared by the two body frames.
  Quaternion relative(prev_base.linear().transpose() * base.linear());
  relative.normalize();
  if (relative.w() < 0) {
    relative.coeffs() *= -1;
  }
  const double sine = relative.vec().norm();
  Vector angular = Vector::Zero();
  if (sine > 1e-12) {
    angular = relative.vec() * (2 * std::atan2(sine, relative.w()) / (sine * dt));
  }
  Twist result{base.linear().transpose() * (base.translation() - prev_base.translation()) / dt,
               angular};
  if (!result.linear.allFinite() || !result.angular.allFinite()) {
    return {};
  }
  return result;
}

struct ImuReading
{
  Vector gyro;
  Vector specific_force;
};

inline ImuReading reorient_livox(const Transform &imu_from_base, const Vector &gyro,
                                 const Vector &acceleration_g)
{
  return {imu_from_base.linear().transpose() * gyro,
          imu_from_base.linear().transpose() * acceleration_g * standard_gravity};
}

enum class TimeStatus { valid, clock_unavailable, timestamp_invalid, non_monotonic, future, stale };

inline TimeStatus check_stamp(int64_t stamp, int64_t previous, int64_t now,
                              int64_t max_age, int64_t future_tolerance)
{
  if (now <= 0) return TimeStatus::clock_unavailable;
  if (stamp <= 0) return TimeStatus::timestamp_invalid;
  if (stamp <= previous) return TimeStatus::non_monotonic;
  if (stamp - now > future_tolerance) return TimeStatus::future;
  if (now - stamp > max_age) return TimeStatus::stale;
  return TimeStatus::valid;
}

inline const char *describe(TimeStatus status)
{
  switch (status) {
    case TimeStatus::valid: return "valid";
    case TimeStatus::clock_unavailable: return "clock_unavailable";
    case TimeStatus::timestamp_invalid: return "timestamp_invalid";
    case TimeStatus::non_monotonic: return "duplicate_or_rollback";
    case TimeStatus::future: return "future_or_clock_domain";
    case TimeStatus::stale: return "stale_or_clock_domain";
  }
  return "unknown";
}
}  // namespace culvert
