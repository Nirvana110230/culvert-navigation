#pragma once

#include <Eigen/Geometry>
#include <json/json.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <chrono>
#include <list>
#include <memory>
#include <string>
#include <vector>

namespace culvert {
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using Tree = pcl::search::KdTree<pcl::PointXYZ>;
using Pose = Eigen::Matrix4d;
using Clock = std::chrono::steady_clock;
inline double milliseconds(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

inline constexpr int min_points = 24;
inline constexpr int max_local_points = 150000;
inline constexpr int max_scan_points = 250000;
inline constexpr int max_map_points = 5000000;
inline constexpr double fine_leaf = 0.10;
inline constexpr double rough_leaf = 0.25;
inline constexpr double inlier_distance = 0.30;
inline constexpr double min_overlap = 0.55;
inline constexpr double max_rmse = 0.18;
inline constexpr double min_weak_ratio = 0.004;
inline constexpr double min_weak_ratio_se3 = 0.0005;
inline constexpr double max_shift = 2.0;
inline constexpr double max_angle_deg = 40.0;

Json::Value number(double value);
template<class Derived> Json::Value matrix(const Eigen::MatrixBase<Derived>& m) {
  Json::Value out(Json::arrayValue);
  for (int r = 0; r < m.rows(); ++r) {
    Json::Value row(Json::arrayValue);
    for (int c = 0; c < m.cols(); ++c) row.append(number(m(r,c)));
    out.append(row);
  }
  return out;
}
template<class Derived> Json::Value vector_json(const Eigen::MatrixBase<Derived>& v) {
  Json::Value result(Json::arrayValue);
  for (int i = 0; i < v.size(); ++i) result.append(number(v(i)));
  return result;
}
struct PreparedGicp;
struct Target {
  Cloud::Ptr rough, fine;
  Tree::Ptr rough_tree, fine_tree;
  std::vector<Eigen::Vector3d> normals;
  std::vector<int8_t> normal_status;
  std::shared_ptr<PreparedGicp> gicp;
  int gicp_threads{};
  Eigen::Vector3d center;
  Eigen::Vector3d min_corner, max_corner;
  double radius{}, grid_m{}, effective_radius{};
};
struct Map {
  Json::Value meta;
  Cloud::Ptr rough, fine;
  Tree::Ptr rough_tree, fine_tree;
  std::list<Target> cache;
};
struct Candidate {
  Pose pose = Pose::Identity();
  bool converged{}, native_ok{};
  std::string reason;
  Json::Value stages{Json::objectValue};
};

std::string sha256(const std::string& path);
Cloud::Ptr read_pcd(const std::string& path, int limit, size_t max_bytes);
Cloud::Ptr voxel(const Cloud::Ptr& points, float leaf);
Tree::Ptr make_tree(const Cloud::Ptr& points);
Target& local_target(Map& map, const Eigen::Vector3d& center, double radius, bool& hit);
Candidate align_icp(const Cloud::Ptr& raw, Target& target, const Pose& initial);
Candidate align_gicp(const Cloud::Ptr& raw, Target& target, const Pose& initial, int threads);
Json::Value evidence(const Cloud::Ptr& source, Target& target, const Pose& pose);
bool valid_pose(const Pose& pose, double eps);
}  // namespace culvert
