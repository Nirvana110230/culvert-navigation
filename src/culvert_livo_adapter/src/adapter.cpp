#include "culvert_livo_adapter/cloud.hpp"
#include "culvert_livo_adapter/kinematics.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <ctime>
#include <map>
#include <memory>
#include <string>
#include <utility>

namespace culvert
{
using Odom = nav_msgs::msg::Odometry;
using Imu = sensor_msgs::msg::Imu;
using Diagnostic = diagnostic_msgs::msg::DiagnosticStatus;
using Steady = std::chrono::steady_clock;
using Stamp = builtin_interfaces::msg::Time;
constexpr int64_t billion = 1000000000LL;

double seconds(Steady::duration elapsed)
{
  return std::chrono::duration<double>(elapsed).count();
}

int64_t nanos(const Stamp &stamp)
{
  if (stamp.sec < 0 || stamp.nanosec >= billion) return 0;
  return static_cast<int64_t>(stamp.sec) * billion + stamp.nanosec;
}

template<typename T>
void set_xyz(T &dst, const Vector &src)
{
  dst.x = src.x();
  dst.y = src.y();
  dst.z = src.z();
}

geometry_msgs::msg::Quaternion rotation(const Transform &pose)
{
  const Quaternion q(pose.linear());
  geometry_msgs::msg::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

geometry_msgs::msg::TransformStamped tf_message(const Transform &pose, const Stamp &stamp,
                                               const std::string &parent, const std::string &child)
{
  geometry_msgs::msg::TransformStamped out;
  out.header.stamp = stamp;
  out.header.frame_id = parent;
  out.child_frame_id = child;
  set_xyz(out.transform.translation, pose.translation());
  out.transform.rotation = rotation(pose);
  return out;
}

struct Timing
{
  std::array<double, 256> recent{};
  uint64_t count = 0;
  double sum = 0, maximum = 0;
  void add(double ms)
  {
    recent[count % recent.size()] = ms;
    ++count;
    sum += ms;
    maximum = std::max(maximum, ms);
  }
  std::pair<double,double> percentiles() const
  {
    if (!count) return {-1, -1};
    const size_t n = std::min<uint64_t>(count, recent.size());
    auto sorted = recent;
    std::sort(sorted.begin(), sorted.begin() + n);
    return {sorted[(n * 50 + 99) / 100 - 1], sorted[(n * 95 + 99) / 100 - 1]};
  }
};

class Adapter final : public rclcpp::Node
{
public:
  Adapter() : Node("culvert_livo_adapter")
  {
    const auto profile = declare_parameter<std::string>("profile", "formal");
    profile_ = profile;
    const bool test = profile == "isolated_test";
    const auto prefix = std::string(test ? "/culvert_test/" : "/culvert/");
    odom_frame_ = test ? "test_odom" : "odom";
    base_frame_ = test ? "test_base_link" : "base_link";
    imu_frame_ = test ? "test_imu" : "culvert_imu";
    const bool verified = declare_parameter<bool>("extrinsics_verified", false);
    calibration_ = declare_parameter<std::string>("calibration_id", "");
    auto read_extrinsic = [this](const std::string &name) {
      const auto t = declare_parameter<std::vector<double>>(name + "_xyz_m", std::vector<double>{});
      const auto q = declare_parameter<std::vector<double>>(name + "_xyzw", std::vector<double>{});
      return extrinsic(t, q);
    };
    const auto ib = read_extrinsic("imu_from_base");
    const auto il = read_extrinsic("imu_from_lidar");
    if (profile != "formal" && !test) config_error_ = "unrecognized_profile";
    else if (!ib || !il) config_error_ = "missing_or_invalid_extrinsics";
    else if ((!test && (!verified || calibration_.empty() || calibration_ == "SYNTHETIC_TEST_ONLY")) ||
             (test && calibration_ != "SYNTHETIC_TEST_ONLY")) config_error_ = "calibration_unverified";
    else {
      imu_from_base_ = *ib;
      base_from_lidar_ = ib->inverse() * *il;
    }
    max_age_ = declare_parameter<double>("max_stamp_age_s", 0.75);
    future_ = declare_parameter<double>("max_future_s", 0.1);
    pair_timeout_ = declare_parameter<double>("pair_timeout_s", 0.25);
    stream_timeout_ = declare_parameter<double>("stream_timeout_s", 0.75);
    min_dt_ = declare_parameter<double>("min_twist_dt_s", 0.001);
    max_dt_ = declare_parameter<double>("max_twist_dt_s", 0.5);
    linear_limit_ = declare_parameter<double>("max_linear_mps", 0.0);
    angular_limit_ = declare_parameter<double>("max_angular_radps", 0.0);
    const int limit = declare_parameter<int>("queue_limit", 8);
    auto bounded = [](double x, double low, double high) {
      return std::isfinite(x) && x >= low && x <= high;
    };
    if (!bounded(max_age_, 0.001, 60) || !bounded(future_, 0, 5) ||
        !bounded(stream_timeout_, 0.05, 60) || !bounded(pair_timeout_, 0.001, 5) ||
        !bounded(min_dt_, 0.000001, 1) || !bounded(max_dt_, min_dt_, 10) ||
        limit < 1 || limit > 32) config_error_ = "invalid_limits";
    if (config_error_.empty() && (!std::isfinite(linear_limit_) || linear_limit_ <= 0 ||
        !std::isfinite(angular_limit_) || angular_limit_ <= 0)) config_error_ = "missing_motion_limits";
    limit_ = static_cast<size_t>(std::clamp(limit, 1, 32));
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        test ? "/culvert_test/diagnostics" : "/diagnostics", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() { diagnose(); });
    if (!config_error_.empty()) {
      RCLCPP_ERROR(get_logger(), "Adapter locked: %s", config_error_.c_str());
      return;
    }
    odom_pub_ = create_publisher<Odom>(prefix + "odom", 5);
    cloud_pub_ = create_publisher<Cloud>(prefix + "localization_cloud", 2);
    imu_pub_ = create_publisher<Imu>(prefix + "imu", 20);
    dynamic_tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    Transform base_from_measurement = Transform::Identity();
    base_from_measurement.translation() = imu_from_base_.inverse().translation();
    static_tf_->sendTransform(tf_message(base_from_measurement, now(), base_frame_, imu_frame_));

    auto source_qos = rclcpp::SensorDataQoS();
    source_qos.keep_last(limit_);
    state_sub_ = create_subscription<Odom>("/culvert_source/state", source_qos,
        [this](Odom::ConstSharedPtr msg) { measure(state_cost_, [&]() { state(*msg); }); });
    cloud_sub_ = create_subscription<Cloud>("/culvert_source/scan", source_qos,
        [this](Cloud::ConstSharedPtr msg) { measure(cloud_cost_, [&]() { scan(std::move(msg)); }); });
    imu_sub_ = create_subscription<Imu>("/livox/imu", rclcpp::SensorDataQoS().keep_last(50),
        [this](Imu::ConstSharedPtr msg) { measure(imu_cost_, [&]() { imu(*msg); }); });
  }

private:
  struct Stream {
    int64_t watermark = 0, last_input_stamp = 0, last_output_stamp = 0;
    Steady::time_point last_input{}, last_output{};
    uint64_t seen = 0, published = 0, consecutive_outputs = 0;
  };
  struct Pending {
    Steady::time_point arrival;
    Cloud::ConstSharedPtr cloud;
    bool odom_ready;
  };
  template<typename F>
  void measure(Timing &metric, F callback)
  {
    const auto start = Steady::now();
    callback();
    const double ms = 1000 * seconds(Steady::now() - start);
    metric.add(ms);
    callback_cost_.add(ms);
  }
  void reject(std::string reason)
  {
    ++rejects_;
    ++rejections_[reason];
    last_reject_ = std::move(reason);
    last_reject_time_ = Steady::now();
  }
  int64_t clock_sample()
  {
    const int64_t current = now().nanoseconds();
    const auto wall = Steady::now();
    if (last_clock_ > 0 && current < last_clock_) clock_rollback_ = true;
    if (current != last_clock_) {
      last_clock_change_ = wall;
      last_clock_ = current;
    }
    const bool paused = get_clock()->ros_time_is_active() &&
                        seconds(wall - last_clock_change_) > stream_timeout_;
    time_status_ = clock_rollback_ ? "clock_rollback_restart_required" :
                   current <= 0 ? "waiting_for_clock" : paused ? "simulation_clock_paused" : "running";
    if (time_status_ != "running") {
      pending_.clear();
      previous_.reset();
      for (auto *s : {&state_, &cloud_, &imu_}) s->consecutive_outputs = 0;
      return 0;
    }
    return current;
  }
  bool admit(Stream &stream, const std::string &name, const Stamp &stamp, bool frame_ok)
  {
    ++stream.seen;
    stream.last_input_stamp = nanos(stamp);
    stream.last_input = Steady::now();
    const int64_t current = clock_sample();
    if (!frame_ok) { reject(name + ":frame_mismatch"); return false; }
    if (!current) { reject(name + ":" + time_status_); return false; }
    const auto check = check_stamp(nanos(stamp), stream.watermark, current,
                                   std::llround(billion * max_age_), std::llround(billion * future_));
    if (check == TimeStatus::valid) return true;
    if (check == TimeStatus::non_monotonic) {
      if (nanos(stamp) == stream.watermark) { ++duplicates_; reject(name + ":duplicate"); }
      else { ++backwards_; reject(name + ":rollback"); }
      return false;
    }
    reject(name + ":" + describe(check));
    return false;
  }
  void published(Stream &stream, int64_t stamp)
  {
    const auto wall = Steady::now();
    if (!stream.published || seconds(wall - stream.last_output) > stream_timeout_ ||
        (stamp - stream.last_output_stamp) * 1e-9 > stream_timeout_) stream.consecutive_outputs = 0;
    ++stream.consecutive_outputs;
    ++stream.published;
    stream.last_output = wall;
    stream.last_output_stamp = stamp;
  }
  void state(const Odom &msg)
  {
    if (!admit(state_, "state", msg.header.stamp,
               msg.header.frame_id == "camera_init" && msg.child_frame_id == "imu")) return;
    const auto &p = msg.pose.pose.position;
    const auto &q = msg.pose.pose.orientation;
    const auto world_from_imu = transform({p.x, p.y, p.z}, Quaternion(q.w, q.x, q.y, q.z));
    if (!world_from_imu) { reject("state:invalid_position_or_quaternion_norm"); return; }
    const int64_t stamp = nanos(msg.header.stamp);
    const Transform base = *world_from_imu * imu_from_base_;
    if (!base.matrix().allFinite()) { reject("state:nonfinite_base"); return; }
    state_.watermark = stamp;
    const auto old_stamp = previous_pose_stamp_;
    const bool lost = previous_ && seconds(Steady::now() - previous_pose_wall_) > stream_timeout_;
    const double dt = double(stamp - old_stamp) / billion;
    const auto twist = previous_ && !lost ? difference(*previous_, base, dt, min_dt_, max_dt_) :
                                           std::optional<Twist>{};
    if (twist && exceeds_motion_limit(*twist, linear_limit_, angular_limit_)) {
      // Preserve the last trusted pose. A persistent step must not silently become a new baseline.
      pose_jump_latched_ = true;
      pending_.clear();
      state_.consecutive_outputs = cloud_.consecutive_outputs = 0;
      reject("state:motion_jump_restart_required");
      return;
    }
    if (pose_jump_latched_) { reject("state:motion_jump_restart_required"); return; }
    previous_ = base;
    previous_pose_stamp_ = stamp;
    previous_pose_wall_ = Steady::now();
    if (!twist) {
      warmup_stamp_ = stamp;
      state_.consecutive_outputs = cloud_.consecutive_outputs = 0;
      pending_.erase(stamp);
      if (old_stamp != 0) reject("state:twist_baseline_reset");
      return;
    }
    Odom out;
    out.header.stamp = msg.header.stamp;
    out.header.frame_id = odom_frame_;
    out.child_frame_id = base_frame_;
    set_xyz(out.pose.pose.position, base.translation());
    out.pose.pose.orientation = rotation(base);
    set_xyz(out.twist.twist.linear, twist->linear);
    set_xyz(out.twist.twist.angular, twist->angular);
    last_dt_ = dt;
    odom_pub_->publish(out);
    dynamic_tf_->sendTransform(tf_message(base, msg.header.stamp, odom_frame_, base_frame_));
    published(state_, stamp);
    pair(stamp, nullptr);
  }
  void scan(Cloud::ConstSharedPtr msg)
  {
    if (!admit(cloud_, "scan", msg->header.stamp, msg->header.frame_id == "livo_lidar")) return;
    if (msg->data.size() > max_cloud_bytes) { reject("scan:oversize"); return; }
    const auto stamp = nanos(msg->header.stamp);
    cloud_.watermark = stamp;
    if (pose_jump_latched_) { reject("scan:motion_jump_restart_required"); return; }
    if (stamp == warmup_stamp_) return;
    pair(stamp, std::move(msg));
  }
  void expire()
  {
    const auto wall = Steady::now();
    for (auto it = pending_.begin(); it != pending_.end();) {
      if (seconds(wall - it->second.arrival) > pair_timeout_) {
        reject(it->second.cloud ? "scan:no_matching_state" : "state:no_matching_scan");
        it = pending_.erase(it);
        ++expired_;
      } else {
        ++it;
      }
    }
  }
  void pair(int64_t stamp, Cloud::ConstSharedPtr incoming)
  {
    expire();
    auto inserted = pending_.try_emplace(stamp, Pending{Steady::now(), nullptr, false});
    auto &item = inserted.first->second;
    if (incoming) item.cloud = std::move(incoming);
    else item.odom_ready = true;
    if (item.cloud && item.odom_ready) {
      Cloud out;
      std::string error;
      // Recheck the measurement clock at both ends of the potentially expensive conversion.
      const auto fresh = [&]() {
        return check_stamp(stamp, last_cloud_output_, clock_sample(),
                           std::llround(billion * max_age_), std::llround(billion * future_)) == TimeStatus::valid;
      };
      const auto source = item.cloud;
      const auto wait_ms = 1000 * seconds(Steady::now() - item.arrival);
      pending_.erase(stamp);
      if (!fresh()) { reject("scan:stale_pair"); return; }
      const auto conversion_start = Steady::now();
      const bool converted = transform_cloud(*source, base_from_lidar_, out, error);
      conversion_cost_.add(1000 * seconds(Steady::now() - conversion_start));
      if (!converted) reject("scan:" + error);
      else if (!fresh()) reject("scan:expired_during_conversion");
      else {
        out.header.frame_id = base_frame_;
        cloud_pub_->publish(out);
        last_cloud_output_ = stamp;
        published(cloud_, stamp);
        last_pair_wait_ms_ = wait_ms;
        pair_wait_.add(wait_ms);
      }
    }
    while (pending_.size() > limit_) {
      pending_.erase(pending_.begin());
      ++overflow_;
      reject("pair:queue_overflow");
    }
  }
  void imu(const Imu &msg)
  {
    if (!admit(imu_, "imu", msg.header.stamp, msg.header.frame_id == "livox_frame")) return;
    const auto &w = msg.angular_velocity;
    const auto &a = msg.linear_acceleration;
    const auto reading = reorient_livox(imu_from_base_, {w.x, w.y, w.z}, {a.x, a.y, a.z});
    if (!reading.gyro.allFinite() || !reading.specific_force.allFinite()) {
      reject("imu:nonfinite");
      return;
    }
    imu_.watermark = nanos(msg.header.stamp);
    Imu out;
    out.header = msg.header;
    out.header.frame_id = imu_frame_;
    out.orientation.w = 1.0;
    out.orientation_covariance[0] = -1.0;
    set_xyz(out.angular_velocity, reading.gyro);
    set_xyz(out.linear_acceleration, reading.specific_force);
    imu_pub_->publish(out);
    published(imu_, nanos(msg.header.stamp));
  }
  void diagnose()
  {
    const auto current = clock_sample();
    expire();
    const auto wall = Steady::now();
    auto wall_age = [&](Steady::time_point t) {
      return t == Steady::time_point{} ? -1.0 : seconds(wall - t);
    };
    auto ros_age = [&](int64_t stamp) {
      return current && stamp > 0 ? (current - stamp) * 1e-9 : -1.;
    };
    auto old = [&](const Stream &s) {
      return s.consecutive_outputs < 2 || wall_age(s.last_output) > stream_timeout_ ||
             check_stamp(s.last_output_stamp, 0, current, std::llround(max_age_ * billion),
                         std::llround(future_ * billion)) != TimeStatus::valid;
    };
    Diagnostic d;
    d.name = "culvert_livo_adapter";
    d.hardware_id = "fast_livo2";
    if (!config_error_.empty()) { d.level = Diagnostic::ERROR; d.message = config_error_; }
    else if (!current) { d.level = Diagnostic::ERROR; d.message = time_status_; }
    else if (pose_jump_latched_) { d.level = Diagnostic::ERROR; d.message = "motion_jump_restart_required"; }
    else if (old(state_) || old(cloud_) || old(imu_)) {
      d.level = seconds(wall - start_) < stream_timeout_ ? Diagnostic::WARN : Diagnostic::ERROR;
      d.message = "waiting_for_fresh_state_scan_or_imu";
    } else if (rejects_ && seconds(wall - last_reject_time_) <= stream_timeout_) {
      d.level = Diagnostic::WARN;
      d.message = last_reject_;
    } else { d.level = Diagnostic::OK; d.message = "active"; }
    auto key = [&](std::string name, std::string value) {
      diagnostic_msgs::msg::KeyValue pair;
      pair.key = std::move(name);
      pair.value = std::move(value);
      d.values.push_back(std::move(pair));
    };
    auto stream_values = [&](const std::string &name, const Stream &s) {
      key(name + "_seen", std::to_string(s.seen));
      key(name + "_published", std::to_string(s.published));
      key(name + "_age_s", std::to_string(wall_age(s.last_output)));
      key(name + "_last_stamp_ns", std::to_string(s.last_input_stamp));
      key(name + "_input_stamp_ns", std::to_string(s.last_input_stamp));
      key(name + "_output_stamp_ns", std::to_string(s.last_output_stamp));
      key(name + "_input_arrival_age_s", std::to_string(wall_age(s.last_input)));
      key(name + "_output_arrival_age_s", std::to_string(wall_age(s.last_output)));
      key(name + "_input_measurement_age_s", std::to_string(ros_age(s.last_input_stamp)));
      key(name + "_output_measurement_age_s", std::to_string(ros_age(s.last_output_stamp)));
      key(name + "_consecutive_outputs", std::to_string(s.consecutive_outputs));
    };
    stream_values("state", state_);
    stream_values("cloud", cloud_);
    stream_values("imu", imu_);
    key("time_status", time_status_);
    key("profile", profile_);
    key("calibration_id", calibration_);
    key("state_input_frames", "camera_init/imu");
    key("cloud_input_frame", "livo_lidar");
    key("imu_input_frame", "livox_frame");
    key("odom_output_frames", odom_frame_ + "/" + base_frame_);
    key("cloud_output_frame", base_frame_);
    key("imu_output_frame", imu_frame_);
    key("twist_linear_source", "base_origin_secant_current_base_axes");
    key("twist_angular_source", "relative_base_pose_shortest_rotation_over_dt");
    key("imu_angular_source", "livox_gyro_raw_rad_per_s_rotated");
    key("max_linear_mps", std::to_string(linear_limit_));
    key("max_angular_radps", std::to_string(angular_limit_));
    key("rejects", std::to_string(rejects_));
    key("duplicate_count", std::to_string(duplicates_));
    key("rollback_count", std::to_string(backwards_));
    key("other_reject_count", std::to_string(rejects_ - backwards_ - duplicates_));
    for (const auto &entry : rejections_) key("reject." + entry.first, std::to_string(entry.second));
    key("last_reject", last_reject_);
    key("pending_pairs", std::to_string(pending_.size()));
    key("pair_expired", std::to_string(expired_));
    key("queue_overflow", std::to_string(overflow_));
    key("matched_pairs", std::to_string(pair_wait_.count));
    key("pair_stamp_diff_ns", pair_wait_.count ? "0" : "unavailable");
    key("pair_last_wait_ms", std::to_string(last_pair_wait_ms_));
    key("twist_interval_s", std::to_string(last_dt_));
    key("covariance_status", "uncalibrated");
    key("imu_bias_status", "raw_uncorrected");
    auto timing = [&](const std::string &name, const Timing &metric) {
      const auto p = metric.percentiles();
      key(name + "_mean_ms", std::to_string(metric.count ? metric.sum / metric.count : -1));
      key(name + "_max_ms", std::to_string(metric.maximum));
      key(name + "_p50_ms", std::to_string(p.first));
      key(name + "_p95_ms", std::to_string(p.second));
    };
    timing("callback_wall", callback_cost_);
    timing("state_callback_wall", state_cost_);
    timing("cloud_callback_wall", cloud_cost_);
    timing("imu_callback_wall", imu_cost_);
    timing("cloud_conversion_wall", conversion_cost_);
    timing("pair_wait", pair_wait_);
    key("process_cpu_ms", std::to_string(1000.0 * (std::clock() - start_cpu_) / CLOCKS_PER_SEC));
    diagnostic_msgs::msg::DiagnosticArray diag;
    diag.header.stamp = now();
    diag.status.push_back(std::move(d));
    diagnostics_->publish(diag);
  }

  std::string config_error_, profile_, calibration_, odom_frame_, base_frame_, imu_frame_;
  std::string time_status_ = "initializing", last_reject_ = "none";
  Transform imu_from_base_ = Transform::Identity(), base_from_lidar_ = Transform::Identity();
  std::optional<Transform> previous_;
  double max_age_, future_, pair_timeout_, stream_timeout_, min_dt_, max_dt_;
  double last_dt_ = 0, linear_limit_ = 0, angular_limit_ = 0, last_pair_wait_ms_ = -1;
  size_t limit_ = 8;
  int64_t last_clock_ = 0, warmup_stamp_ = 0, last_cloud_output_ = 0, previous_pose_stamp_ = 0;
  bool clock_rollback_ = false, pose_jump_latched_ = false;
  uint64_t rejects_ = 0, expired_ = 0, overflow_ = 0, duplicates_ = 0, backwards_ = 0;
  Steady::time_point start_ = Steady::now(), last_clock_change_ = start_, last_reject_time_{}, previous_pose_wall_{};
  std::clock_t start_cpu_ = std::clock();
  Stream state_, cloud_, imu_;
  Timing state_cost_, cloud_cost_, imu_cost_, callback_cost_, conversion_cost_, pair_wait_;
  std::map<std::string, uint64_t> rejections_;
  std::map<int64_t, Pending> pending_;
  rclcpp::Publisher<Odom>::SharedPtr odom_pub_;
  rclcpp::Publisher<Cloud>::SharedPtr cloud_pub_;
  rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Subscription<Odom>::SharedPtr state_sub_;
  rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
};
}  // namespace culvert

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<culvert::Adapter>());
  rclcpp::shutdown();
}
