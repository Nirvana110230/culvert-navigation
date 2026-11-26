#include "registration.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <set>

namespace culvert {
namespace {
double quantile(const std::vector<double>& sorted, double q) {
  if (sorted.empty()) return NAN;
  const double index=q*double(sorted.size()-1);
  const auto lo=static_cast<size_t>(index);
  return sorted[lo]+(index-lo)*(sorted[static_cast<size_t>(std::ceil(index))]-sorted[lo]);
}

// Reject lines and corners whose local PCA cannot determine a plane reliably.
bool normal_at(Target& target, int index, Eigen::Vector3d& n) {
  if (target.normal_status[index]==0) {
    std::vector<int> ids(12);
    std::vector<float> d2(12);
    target.normal_status[index]=-1;
    if (target.fine_tree->nearestKSearch((*target.fine)[index],12,ids,d2)==12 && d2.back()<=1.0) {
      Eigen::Vector3d mean=Eigen::Vector3d::Zero();
      for (int i : ids) mean+=(*target.fine)[i].getVector3fMap().cast<double>();
      mean/=ids.size();
      Eigen::Matrix3d s=Eigen::Matrix3d::Zero();
      for (int i : ids) {
        const Eigen::Vector3d d=(*target.fine)[i].getVector3fMap().cast<double>()-mean;
        s.noalias()+=d*d.transpose();
      }
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> e(s);
      if (e.info()==Eigen::Success) {
        const auto vals=e.eigenvalues();
        if (vals(1)>1e-9 && vals(1)>=0.08*vals(2) && vals(0)<=0.25*vals(1)) {
          target.normal_status[index]=1;
          target.normals[index]=e.eigenvectors().col(0);
        }
      }
    }
  }
  if (target.normal_status[index]<0) return false;
  n=target.normals[index];
  return true;
}

using Cell = std::array<int64_t,3>;
Cell cell(const Eigen::Vector3d& p) {
  return {int64_t(std::floor(p.x())),int64_t(std::floor(p.y())),int64_t(std::floor(p.z()))};
}
struct Plane { Eigen::Vector3d lever, normal; };
}

Json::Value evidence(const Cloud::Ptr& source, Target& target, const Pose& pose) {
  Json::Value out;
  std::vector<double> distances, inside;
  std::vector<Plane> planes;
  std::set<Cell> support_cells, inlier_cells;
  std::vector<int> ids(1);
  std::vector<float> d2(1);
  int no_neighbor=0;
  double sum2=0, lever2=0, lever3=0;
  const Eigen::Vector3d origin=pose.topRightCorner<3,1>();
  for (const auto& p : *source) {
    const Eigen::Vector3d lever=pose.topLeftCorner<3,3>()*p.getVector3fMap().cast<double>();
    const Eigen::Vector3d world=lever+origin;
    const auto bin=cell(world);
    support_cells.insert(bin);
    if (target.fine_tree->nearestKSearch(pcl::PointXYZ(world.x(),world.y(),world.z()),1,ids,d2)!=1 ||
        !std::isfinite(d2.front()) || d2.front()<0) { ++no_neighbor; continue; }
    const double d=std::sqrt(d2.front());
    distances.push_back(d);
    if (d>inlier_distance) continue;
    inside.push_back(d);
    sum2+=d2.front();
    inlier_cells.insert(bin);
    Eigen::Vector3d n;
    if (normal_at(target,ids[0],n)) {
      planes.push_back({lever,n});
      lever2+=lever.head<2>().squaredNorm();
      lever3+=lever.squaredNorm();
    }
  }
  std::sort(distances.begin(),distances.end());
  std::sort(inside.begin(),inside.end());
  out["source_finite_denominator"]=Json::UInt64(source->size());
  out["matched_nearest"]=Json::UInt64(distances.size());
  out["inliers"]=Json::UInt64(inside.size());
  out["no_nearest"]=no_neighbor;
  out["distance_outliers"]=Json::UInt64(distances.size()-inside.size());
  out["without_reliable_normal"]=Json::UInt64(inside.size()-planes.size());
  out["distance_threshold_m"]=inlier_distance;
  out["inlier_ratio"]=double(inside.size())/source->size();
  out["rmse_m"]=number(inside.empty() ? NAN : std::sqrt(sum2/inside.size()));
  out["all_nearest_median_m"]=number(quantile(distances,0.50));
  out["all_nearest_p95_m"]=number(quantile(distances,0.95));
  out["all_nearest_max_m"]=number(distances.empty() ? NAN : distances.back());
  out["inlier_median_m"]=number(quantile(inside,0.50));
  out["inlier_p95_m"]=number(quantile(inside,0.95));
  out["inlier_max_m"]=number(inside.empty() ? NAN : inside.back());
  out["cells_1m"]=Json::UInt64(support_cells.size());
  out["inlier_cells_1m"]=Json::UInt64(inlier_cells.size());
  out["cell_coverage"]=double(inlier_cells.size())/support_cells.size();
  out["plane_support"]=Json::UInt64(planes.size());
  const double scale=std::max(0.1,std::sqrt(lever2/std::max<size_t>(planes.size(),1)));
  const double scale3=std::max(0.1,std::sqrt(lever3/std::max<size_t>(planes.size(),1)));
  Eigen::Matrix3d H=Eigen::Matrix3d::Zero();
  using Vector6=Eigen::Matrix<double,6,1>;
  using Matrix6=Eigen::Matrix<double,6,6>;
  Matrix6 H6=Matrix6::Zero();
  for (const auto& p : planes) {
    const Eigen::Vector3d j(p.normal.x(),p.normal.y(),p.lever.cross(p.normal).z()/scale);
    H.noalias()+=j*j.transpose();
    Vector6 j6;
    j6 << p.normal, p.lever.cross(p.normal)/scale3;
    H6.noalias()+=j6*j6.transpose();
  }
  H/=std::max<size_t>(planes.size(),1);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(H);
  out["scale_m"]=scale;
  out["H_xy_scaled_yaw"]=matrix(H);
  out["weak_ratio"]=0.0;
  if (eig.info()==Eigen::Success && !planes.empty()) {
    out["eigenvalues_ascending"]=vector_json(eig.eigenvalues());
    if (eig.eigenvalues()(2)>1e-12) {
      out["weak_ratio"]=std::max(0.0,eig.eigenvalues()(0))/eig.eigenvalues()(2);
      out["weak_direction_xy_scaled_yaw"]=vector_json(eig.eigenvectors().col(0));
    }
  }
  H6/=std::max<size_t>(planes.size(),1);
  Eigen::SelfAdjointEigenSolver<Matrix6> eig6(H6);
  out["scale_se3_m"]=scale3;
  out["H_se3_scaled"]=matrix(H6);
  out["weak_ratio_se3"]=0.0;
  out["condition_se3"]=Json::nullValue;
  out["weak_direction_se3"]=Json::nullValue;
  out["singular_se3"]=true;
  if (eig6.info()==Eigen::Success && !planes.empty()) {
    const auto vals=eig6.eigenvalues().cwiseMax(0.0).eval();
    out["eigenvalues_se3_ascending"]=vector_json(vals);
    if (vals(5)>1e-12) {
      out["weak_ratio_se3"]=vals(0)/vals(5);
      out["weak_direction_se3"]=vector_json(eig6.eigenvectors().col(0));
      if (vals(0)>1e-12*vals(5)) {
        out["singular_se3"]=false;
        out["condition_se3"]=vals(5)/vals(0);
      }
    }
  }
  return out;
}
}  // namespace culvert
