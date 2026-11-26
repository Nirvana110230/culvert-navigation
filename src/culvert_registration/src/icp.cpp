#include "registration.hpp"

#include <pcl/registration/icp.h>

namespace culvert {
class MeasuredICP : public pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> {
public:
  int iterations() const { return nr_iterations_; }
};

Candidate align_icp(const Cloud::Ptr& raw, Target& target, const Pose& initial) {
  Candidate result;
  result.pose=initial;
  result.stages["rough"]=Json::nullValue;
  result.stages["fine"]=Json::nullValue;
  for (bool rough : {true,false}) {
    auto start=Clock::now();
    const char* name=rough ? "rough" : "fine";
    auto& log=result.stages[name];
    const auto& dst=rough ? target.rough : target.fine;
    const double leaf=rough ? rough_leaf : fine_leaf;
    auto src=voxel(raw,leaf);
    log["source_points"]=Json::UInt64(src->size());
    log["target_points"]=Json::UInt64(dst->size());
    log["leaf_m"]=leaf;
    log["max_iterations"]=rough ? 5 : 10;
    log["mse_limit_m2"]=rough ? 0.2 : 0.1;
    if (src->size()<min_points) { result.reason="insufficient_scan"; return result; }
    MeasuredICP solver;
    solver.setMaximumIterations(rough ? 5 : 10);
    solver.setInputTarget(dst);
    solver.setSearchMethodTarget(rough ? target.rough_tree : target.fine_tree, true);
    solver.setInputSource(src);
    Cloud aligned;
    solver.align(aligned,result.pose.cast<float>());
    result.pose=solver.getFinalTransformation().cast<double>();
    const bool conv=solver.hasConverged();
    const double mse=valid_pose(result.pose,1e-3) ? solver.getFitnessScore() : NAN;
    log["T_target_source"]=matrix(result.pose);
    log["converged"]=conv;
    log["iterations"]=solver.iterations();
    log["mse_m2"]=number(mse);
    log["elapsed_ms"]=milliseconds(start);
    if (!conv || !std::isfinite(mse) || mse>log["mse_limit_m2"].asDouble()) {
      result.reason=std::string("icp_")+name+(conv ? "_fitness" : "_not_converged");
      result.converged=!rough && conv;
      return result;
    }
  }
  result.converged=result.native_ok=true;
  return result;
}
}  // namespace culvert
