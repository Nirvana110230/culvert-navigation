#include "registration.hpp"

#include <small_gicp/registration/registration_helper.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>
#include <omp.h>

namespace culvert {
struct PreparedGicp {
  small_gicp::PointCloud::Ptr points;
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> tree;
};
static PreparedGicp prepare(const Cloud& cloud, int threads) {
  std::vector<Eigen::Vector3f> points;
  points.reserve(cloud.size());
  for (const auto& p : cloud) points.emplace_back(p.getVector3fMap());
  if (threads==1) {
    auto [result,tree]=small_gicp::preprocess_points(points, fine_leaf, 12, 1);
    return {result,tree};
  }
  // The pinned helper omits the thread argument when it calls voxelgrid_sampling_omp,
  // whose own default is four. Pass the requested count through every parallel stage.
  auto result=small_gicp::voxelgrid_sampling_omp(small_gicp::PointCloud(points),fine_leaf,threads);
  auto tree=std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(result);
  small_gicp::estimate_normals_covariances_omp(*result,*tree,12,threads);
  return {result,tree};
}
Candidate align_gicp(const Cloud::Ptr& raw, Target& target, const Pose& initial, int threads) {
  Candidate out;
  out.pose=initial;
  auto& log=out.stages["gicp"];
  omp_set_dynamic(0);
  omp_set_num_threads(threads);
  auto start=Clock::now();
  log["target_cache_hit"]=bool(target.gicp && target.gicp_threads==threads);
  if (!log["target_cache_hit"].asBool()) {
    target.gicp=std::make_shared<PreparedGicp>(prepare(*target.fine,threads));
    target.gicp_threads=threads;
  }
  log["target_preprocess_ms"]=milliseconds(start);
  start=Clock::now();
  const auto src=prepare(*raw,threads);
  log["source_preprocess_ms"]=milliseconds(start);
  log["source_points"]=Json::UInt64(src.points->size());
  log["target_points"]=Json::UInt64(target.gicp->points->size());
  log["voxel_m"]=fine_leaf;
  log["covariance_neighbors"]=12;
  small_gicp::RegistrationSetting params;
  params.type=small_gicp::RegistrationSetting::GICP;
  params.num_threads=threads;
  params.max_iterations=24;
  params.max_correspondence_distance=1.0;
  log["max_iterations"]=params.max_iterations;
  log["max_correspondence_m"]=params.max_correspondence_distance;
  if (src.points->size()<min_points || target.gicp->points->size()<min_points) {
    out.reason="insufficient_preprocessed_points";
    return out;
  }
  start=Clock::now();
  auto r=small_gicp::align(*target.gicp->points,*src.points,*target.gicp->tree,Eigen::Isometry3d(initial),params);
  log["align_ms"]=milliseconds(start);
  log["iterations_native"]=Json::UInt64(r.iterations);
  log["iterations_executed"]=Json::UInt64(r.iterations+1);
  log["native_inliers"]=Json::UInt64(r.num_inliers);
  log["native_error"]=number(r.error);
  log["native_H"]=matrix(r.H);
  log["native_b"]=vector_json(r.b);
  log["converged"]=r.converged;
  out.pose=r.T_target_source.matrix();
  out.converged=r.converged;
  out.native_ok=r.converged && r.num_inliers>=min_points && std::isfinite(r.error) && r.H.allFinite();
  if (!out.native_ok) out.reason=r.converged ? "gicp_invalid_support" : "gicp_not_converged";
  return out;
}
}  // namespace culvert
