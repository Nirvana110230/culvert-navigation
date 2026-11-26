#include "culvert_livo_adapter/kinematics.hpp"
#include "culvert_livo_adapter/cloud.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

using namespace culvert;
namespace
{
const double pi = std::acos(-1.0);
Transform pose(const Vector &t, const Vector &axis = Vector::UnitZ(), double angle = 0)
{
  return *transform(t, Quaternion(Eigen::AngleAxisd(angle, axis)));
}
void expect_vector(const Vector &actual, const Vector &expected, double tolerance = 1e-10)
{
  for (int i = 0; i < 3; ++i) EXPECT_NEAR(actual[i], expected[i], tolerance);
}
Cloud fixture_cloud()
{
  Cloud c;
  c.width = c.height = 1;
  c.point_step = 16;
  c.row_step = 20;
  c.data.resize(c.row_step);
  const std::array<std::string, 4> names{"z", "intensity", "x", "y"};
  const std::array<float, 4> data{-1, 42, 2, 3};
  for (unsigned i = 0; i < 4; ++i) {
    Field field;
    field.name = names[i];
    field.count = 1;
    field.datatype = Field::FLOAT32;
    field.offset = i * sizeof(float);
    c.fields.push_back(field);
  }
  std::memcpy(c.data.data(), data.data(), sizeof(data));
  return c;
}

TEST(Extrinsics, ChainedBodyPoseAndInverseCloud)
{
  // IMU is yawed 90 degrees in the world, base is rolled 90 degrees in IMU.
  const auto wi = pose({4, 5, 6}, Vector::UnitZ(), pi/2);
  const auto ib = pose({2, 0, 1}, Vector::UnitX(), pi/2);
  const auto il = pose({1, 2, 3}, Vector::UnitZ(), pi/2);
  const auto wb = wi * ib;
  expect_vector(wb.translation(), {4, 7, 7});
  expect_vector(wb.linear() * Vector::UnitY(), {0, 0, 1});
  const Transform bl = ib.inverse() * il;
  const Vector lidar(2, 3, -1);
  // T_I_L p = (-2,4,2); undo T_I_B by subtracting (2,0,1), then Rx(-90).
  expect_vector(bl * lidar, {-4, 1, -4});
  expect_vector(bl.inverse() * Vector(-4, 1, -4), lidar);
  expect_vector(wb * Vector(-4, 1, -4), wi * (il * lidar));
}

TEST(Imu, TiltDegreesAndGravityUnits)
{
  const auto ib = pose({1, 2, 3}, Vector::UnitX(), pi/2);
  const auto reading = reorient_livox(ib, {0, 0, pi/2}, {0, 0, 1});
  expect_vector(reading.gyro, {0, pi/2, 0});
  expect_vector(reading.specific_force, {0, 9.80665, 0});
}

TEST(Pose, QuaternionNormalizationAndSign)
{
  const auto a = transform({0, 0, 0}, Quaternion(.7, 0, 0, .7));
  const auto b = transform({0, 0, 0}, Quaternion(-.7, 0, 0, -.7));
  ASSERT_TRUE(a && b);
  expect_vector(a->linear() * Vector::UnitX(), {0,1,0});
  expect_vector(b->linear() * Vector::UnitX(), {0,1,0});
  const auto delta = difference(*a, *b, 0.1, .001, .5);
  ASSERT_TRUE(delta);
  expect_vector(delta->angular, Vector::Zero());
}

TEST(Twist, ShortRotationAcrossPlusMinus180)
{
  const auto a = pose(Vector::Zero(), Vector::UnitZ(), 179 * pi/180);
  const auto b = pose(Vector::Zero(), Vector::UnitZ(), -179 * pi/180);
  const auto delta = difference(a, b, .1, .001, .5);
  ASSERT_TRUE(delta);
  expect_vector(delta->angular, {0, 0, 20*pi/180});
  expect_vector(delta->linear, Vector::Zero());
}

TEST(Twist, TurningAboutImuGivesBaseLeverArmVelocity)
{
  const Transform ib = pose({1, 0, 0});
  const auto a = pose(Vector::Zero()) * ib;
  const auto b = pose(Vector::Zero(), Vector::UnitZ(), pi/2) * ib;
  const auto d = difference(a, b, .5, .001, .5);
  ASSERT_TRUE(d);
  // World chord (-1,1,0)/0.5 projected into the new 90-degree body.
  expect_vector(d->linear, {2, 2, 0});
  expect_vector(d->angular, {0, 0, pi});
}

TEST(Twist, BaseTurnInPlaceAndWorldToBodyVelocity)
{
  const Transform ib = pose({1, 0, 0});
  const auto a = pose({-1, 0, 0}) * ib;
  const auto b = pose({0, -1, 0}, Vector::UnitZ(), pi/2) * ib;
  auto turn = difference(a, b, .5, .001, .5);
  ASSERT_TRUE(turn);
  expect_vector(turn->linear, Vector::Zero());
  expect_vector(turn->angular, {0,0,pi});
  const auto c = pose({1, -1, 0}, Vector::UnitZ(), pi/2) * ib;
  auto straight = difference(b, c, .5, .001, .5);
  ASSERT_TRUE(straight);
  expect_vector(straight->linear, {0,-2,0});
  expect_vector(straight->angular, Vector::Zero());
}

TEST(Time, OutOfOrderFutureExpiredMissingAndInvalidDt)
{
  EXPECT_EQ(check_stamp(990, 980, 1000, 10, 5), TimeStatus::valid);
  EXPECT_EQ(check_stamp(990, 990, 1000, 10, 5), TimeStatus::non_monotonic);
  EXPECT_EQ(check_stamp(989, 990, 1000, 10, 5), TimeStatus::non_monotonic);
  EXPECT_EQ(check_stamp(989, 0, 1000, 10, 5), TimeStatus::stale);
  EXPECT_EQ(check_stamp(1006, 0, 1000, 10, 5), TimeStatus::future);
  EXPECT_EQ(check_stamp(0, 0, 1000, 10, 5), TimeStatus::timestamp_invalid);
  EXPECT_EQ(check_stamp(1000, 0, 0, 10, 5), TimeStatus::clock_unavailable);
  for (double dt : {0., -.1, .0001, .51, std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_FALSE(difference(Transform::Identity(), Transform::Identity(), dt, .001, .5));
  }
}

TEST(Extrinsics, MissingInvalidOrNonfinite)
{
  EXPECT_FALSE(extrinsic({}, {}));
  EXPECT_FALSE(extrinsic({0, 0, 0}, {0, 0, 0, 0}));
  EXPECT_FALSE(extrinsic({0, 0, 0}, {0, 0, 0, 10}));
  EXPECT_FALSE(extrinsic({0, 0, 0}, {0, 0, 0, std::numeric_limits<double>::infinity()}));
  EXPECT_FALSE(extrinsic({0, std::numeric_limits<double>::quiet_NaN(), 0}, {0,0,0,1}));
}

TEST(Twist, RejectJumpAgainstExplicitLimits)
{
  const Twist t{{3, 4, 0}, {0, 0, 2}};
  EXPECT_TRUE(exceeds_motion_limit(t, 4.99, 2));
  EXPECT_TRUE(exceeds_motion_limit(t, 5, 1.99));
  EXPECT_FALSE(exceeds_motion_limit(t, 5, 2));
}

TEST(Cloud, ActualWireLayoutRotationTranslationIntensityAndInverse)
{
  const auto source = fixture_cloud();
  const auto bl = pose({1,-2,4}, Vector::UnitZ(), pi/2);
  Cloud dst, recovered;
  std::string error;
  ASSERT_TRUE(transform_cloud(source, bl, dst, error)) << error;
  ASSERT_EQ(dst.data.size(), size_t(16));
  std::array<float,4> values;
  std::memcpy(values.data(), dst.data.data(), sizeof(values));
  expect_vector({values[0], values[1], values[2]}, {-2,0,3}, 1e-6);
  EXPECT_EQ(values[3], 42.0f);
  ASSERT_TRUE(transform_cloud(dst, bl.inverse(), recovered, error));
  std::memcpy(values.data(), recovered.data.data(), sizeof(values));
  expect_vector({values[0], values[1], values[2]}, {2,3,-1}, 1e-6);
  EXPECT_EQ(values[3], 42.0f);
}

TEST(Cloud, RejectMalformedOffsetsShortDataAndNonfinitePoints)
{
  Cloud dst;
  std::string error;
  auto src = fixture_cloud();
  src.fields[0].offset = 16;
  EXPECT_FALSE(transform_cloud(src, Transform::Identity(), dst, error));
  src = fixture_cloud();
  src.data.pop_back();
  EXPECT_FALSE(transform_cloud(src, Transform::Identity(), dst, error));
  src = fixture_cloud();
  float invalid = std::numeric_limits<float>::quiet_NaN();
  std::memcpy(src.data.data(), &invalid, sizeof(float));
  EXPECT_FALSE(transform_cloud(src, Transform::Identity(), dst, error));
}
}  // namespace
