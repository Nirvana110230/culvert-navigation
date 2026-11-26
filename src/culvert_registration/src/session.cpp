#include "registration.hpp"

#include <sys/resource.h>
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace culvert {
namespace {
void require(bool condition, const std::string& reason) {
  if (!condition) throw std::runtime_error(reason);
}
std::string field(const Json::Value& req, const char* key) {
  const auto& v=req[key];
  require(v.isString() && !v.asString().empty(),"invalid_"+std::string(key));
  return v.asString();
}
double scalar(const Json::Value& v, const std::string& name) {
  require(v.isNumeric() && std::isfinite(v.asDouble()),"invalid_"+name);
  return v.asDouble();
}
double parameter(const Json::Value& r,const char* key,double fallback,double lo,double hi) {
  if (!r.isMember(key)) return fallback;
  double v=scalar(r[key],key);
  require(v>=lo && v<=hi,"invalid_"+std::string(key));
  return v;
}
Json::Int64 integer(const Json::Value& req, const char* key) {
  require(req[key].isIntegral() && req[key].isInt64(),"invalid_"+std::string(key));
  return req[key].asInt64();
}
Pose read_pose(const Json::Value& v, const char* name) {
  require(v.isArray() && v.size()==4,"invalid_"+std::string(name));
  Pose p;
  for (int r=0;r<4;++r) {
    require(v[r].isArray() && v[r].size()==4,"invalid_"+std::string(name));
    for (int c=0;c<4;++c) p(r,c)=scalar(v[r][c],name);
  }
  require(valid_pose(p,1e-5) && p.topRightCorner<3,1>().norm()<1e6,"invalid_se3_"+std::string(name));
  return p;
}
Json::Value defaults(Json::UInt64 generation) {
  Json::Value out;
  out["generation"]=generation;
  out["accepted"]=false;
  out["converged"]=false;
  out["ok"]=false;
  for (const auto* k : {"T_target_source","T_map_W","rmse","inlier_ratio","weak_ratio"}) out[k]=Json::nullValue;
  return out;
}
double cpu_ms() {
  struct rusage ru{};
  getrusage(RUSAGE_SELF,&ru);
  return 1000.0*(ru.ru_utime.tv_sec+ru.ru_stime.tv_sec)+0.001*(ru.ru_utime.tv_usec+ru.ru_stime.tv_usec);
}
long maxrss() {
  struct rusage ru{};
  getrusage(RUSAGE_SELF,&ru);
  return ru.ru_maxrss;
}
Cloud::Ptr read_scan(const Json::Value& req) {
  require(req.isMember("scan_path")!=req.isMember("points"),"one_scan_input_required");
  if (req.isMember("scan_path")) return read_pcd(field(req,"scan_path"),max_scan_points,64ULL<<20);
  const auto& a=req["points"];
  require(a.isArray() && a.size()<=max_scan_points,"invalid_points");
  Cloud::Ptr c(new Cloud);
  c->reserve(a.size());
  for (const auto& row : a) {
    require(row.isArray() && row.size()==3,"invalid_xyz_row");
    Eigen::Vector3f v;
    for (int i=0;i<3;++i) {
      double x=scalar(row[i],"point");
      require(std::abs(x)<1e6,"xyz_out_of_range");
      v(i)=float(x);
    }
    c->push_back(pcl::PointXYZ(v.x(),v.y(),v.z()));
  }
  return c;
}

class SessionState {
  Map map_;
  Json::UInt64 generation_{}, evictions_{};
  void load(const Json::Value& req,Json::Value& out) {
    Map next;
    auto start=Clock::now();
    const std::string path=field(req,"path");
    const auto expected=field(req,"sha256");
    require(expected.size()==64 && std::all_of(expected.begin(),expected.end(),
      [](unsigned char c){return std::isdigit(c) || (c>='a' && c<='f');}),"invalid_sha256");
    for (const auto* k : {"id","version","frame","units"}) next.meta[k]=field(req,k);
    require(next.meta["units"]=="m","map_units_m_required");
    for (const auto* k : {"gravity","origin"}) {
      const auto& v=req[k];
      require((v.isString() && !v.asString().empty()) || ((v.isObject() || v.isArray()) && v.size()>0),
              "invalid_"+std::string(k));
      next.meta[k]=v;
    }
    require(std::filesystem::is_regular_file(path) && std::filesystem::file_size(path)<=(512ULL<<20),
            "map_file_missing_or_too_large");
    require(sha256(path)==expected,"map_sha256_mismatch");
    auto raw=read_pcd(path,max_map_points,512ULL<<20);
    require(raw->size()>=min_points,"insufficient_map");
    require(sha256(path)==expected,"map_changed_during_load");
    next.meta["path"]=path; next.meta["sha256"]=expected;
    next.meta["raw_points"]=Json::UInt64(raw->size());
    next.meta["raw_aabb_min_m"]=vector_json(bounds(*raw,true));
    next.meta["raw_aabb_max_m"]=vector_json(bounds(*raw,false));
    next.fine=voxel(raw,fine_leaf);
    next.rough=voxel(raw,rough_leaf);
    require(next.rough->size()>=min_points && next.fine->size()>=min_points,"insufficient_map");
    next.fine_tree=make_tree(next.fine);
    next.rough_tree=make_tree(next.rough);
    next.meta["fine_points"]=Json::UInt64(next.fine->size());
    next.meta["rough_points"]=Json::UInt64(next.rough->size());
    next.meta["preprocess_ms"]=milliseconds(start);
    map_=std::move(next);
    evictions_=0;
    out["generation"]=++generation_;
    out["map"]=map_.meta;
    out["reason"]="loaded";
    out["ok"]=true;
  }
  static Eigen::Vector3d bounds(const Cloud& cloud, bool minimum) {
    Eigen::Vector3d b=cloud.front().getVector3fMap().cast<double>();
    for(const auto& p: cloud) {
      const Eigen::Vector3d q=p.getVector3fMap().cast<double>();
      if(minimum) b=b.cwiseMin(q); else b=b.cwiseMax(q);
    }
    return b;
  }
  void match(const Json::Value& req, Json::Value& out, Clock::time_point started) {
    require(generation_>0,"map_unloaded");
    const auto wanted=integer(req,"generation");
    const std::string backend=field(req,"backend");
    require(backend=="icp" || backend=="gicp","unknown_backend");
    const auto requested=req.isMember("threads") ? integer(req,"threads") : 1;
    require(requested>=1 && requested<=8,"invalid_threads");
    const auto source_frame=field(req,"source_frame");
    require(source_frame!=map_.meta["frame"].asString(),"source_equals_map_frame");
    const auto stamp=integer(req,"stamp_ns");
    require(stamp>=0,"invalid_stamp_ns");
    const auto initial=read_pose(req["initial"],"initial"), local=read_pose(req["local"],"local");
    const double radius=parameter(req,"radius",20,0.1,200);
    const double budget=parameter(req,"budget_ms",1000,0.000001,3600000);
    out["scan_id"]=field(req,"scan_id");
    out["stamp_ns"]=stamp;
    out["map_id"]=map_.meta["id"]; out["map_version"]=map_.meta["version"];
    out["map_sha256"]=map_.meta["sha256"]; out["target_frame"]=map_.meta["frame"];
    out["source_frame"]=source_frame; out["backend"]=backend;
    out["initial"]=matrix(initial); out["local"]=matrix(local);
    out["threads_requested"]=Json::Int64(requested);
    out["threads_effective"]=backend=="icp" ? 1 : int(requested);
    out["budget_ms"]=budget;
    out["requested_generation"]=wanted;
    out["ok"]=true;
    if (wanted<1 || Json::UInt64(wanted)!=generation_) {out["reason"]="stale_generation"; return;}
    double age=0, max_age=0;
    require(req.isMember("now_ns")==req.isMember("max_age_ms"),"age_parameters_must_be_paired");
    if (req.isMember("now_ns")) {
      auto now=integer(req,"now_ns");
      require(now>=0,"invalid_now_ns");
      max_age=parameter(req,"max_age_ms",0,0,3600000);
      age=(now-stamp)/1e6;
      out["age_at_request_ms"]=age;
      if (age<0 || age>max_age) {out["reason"]="stale_stamp"; return;}
    }
    auto scan_start=Clock::now();
    auto raw=read_scan(req);
    out["scan_read_ms"]=milliseconds(scan_start);
    out["scan_points"]=Json::UInt64(raw->size());
    if (raw->size()<min_points) {out["reason"]="insufficient_scan"; return;}
    bool hit=false;
    const auto previous=map_.cache.size();
    auto start=Clock::now();
    const Eigen::Vector3d center=initial.topRightCorner<3,1>();
    out["submap"]["center_m"]=vector_json(center);
    out["submap"]["radius_m"]=radius;
    Target& target=local_target(map_,center,radius,hit);
    if(previous==2 && !hit) ++evictions_;
    auto& sub=out["submap"];
    sub["cache_center_m"]=vector_json(target.center);
    sub["grid_m"]=target.grid_m; sub["effective_radius_m"]=target.effective_radius;
    sub["aabb_min_m"]=vector_json(target.min_corner); sub["aabb_max_m"]=vector_json(target.max_corner);
    sub["cached"]=hit; sub["generation"]=generation_; sub["entries"]=Json::UInt64(map_.cache.size());
    sub["evictions"]=evictions_;
    sub["rough_points"]=Json::UInt64(target.rough->size());
    sub["fine_points"]=Json::UInt64(target.fine->size());
    sub["lookup_ms"]=milliseconds(start);
    if (milliseconds(started)>budget) {out["reason"]="time_budget"; return;}
    start=Clock::now();
    auto candidate=backend=="icp" ? align_icp(raw,target,initial) : align_gicp(raw,target,initial,int(requested));
    out["backend_ms"]=milliseconds(start);
    out["stages"]=candidate.stages; out["converged"]=candidate.converged;
    if (!valid_pose(candidate.pose,1e-3)) {out["reason"]="invalid_result_se3"; return;}
    // PCL float SVD matrices can accumulate roundoff; reproject only validated near-rigid results.
    Eigen::Quaterniond q(candidate.pose.topLeftCorner<3,3>());
    candidate.pose.topLeftCorner<3,3>()=q.normalized().toRotationMatrix();
    const auto& p=candidate.pose;
    out["T_target_source"]=matrix(p);
    out["T_map_W"]=matrix(p*local.inverse());
    const double delta=(p.topRightCorner<3,1>()-center).norm();
    const double angle=Eigen::AngleAxisd(initial.topLeftCorner<3,3>().transpose()*p.topLeftCorner<3,3>()).angle()*180.0/std::acos(-1.0);
    out["correction_translation_m"]=delta; out["correction_angle_deg"]=angle;
    start=Clock::now();
    auto quality=evidence(raw,target,p);
    out["evaluation_ms"]=milliseconds(start);
    out["quality"]=quality;
    out["rmse"]=quality["rmse_m"]; out["inlier_ratio"]=quality["inlier_ratio"]; out["weak_ratio"]=quality["weak_ratio"];
    out["weak_ratio_se3"]=quality["weak_ratio_se3"];
    auto& limits=out["limits"];
    limits["overlap_min"]=min_overlap; limits["rmse_max_m"]=max_rmse; limits["weak_ratio_min"]=min_weak_ratio;
    limits["weak_ratio_se3_min"]=min_weak_ratio_se3;
    limits["inliers_min"]=min_points; limits["plane_support_min"]=min_points;
    limits["coverage_min"]=0.4; limits["supported_cells_min"]=3;
    limits["shift_max_m"]=max_shift; limits["rotation_max_deg"]=max_angle_deg;
    Json::Value failures(Json::arrayValue);
    if (!candidate.native_ok) failures.append(candidate.reason);
    if (quality["inliers"].asUInt64()<min_points || quality["inlier_ratio"].asDouble()<min_overlap) failures.append("low_overlap");
    if (!out["rmse"].isNumeric() || out["rmse"].asDouble()>max_rmse) failures.append("high_rmse");
    if (quality["inlier_cells_1m"].asUInt64()<3 || quality["cell_coverage"].asDouble()<0.4) failures.append("low_coverage");
    if (quality["plane_support"].asUInt64()<min_points || out["weak_ratio"].asDouble()<min_weak_ratio) failures.append("weak_geometry");
    if (quality["weak_ratio_se3"].asDouble()<min_weak_ratio_se3) failures.append("weak_se3");
    if (delta>max_shift || angle>max_angle_deg) failures.append("excessive_correction");
    if (milliseconds(started)>budget) failures.append("time_budget");
    if (req.isMember("now_ns") && age+milliseconds(started)>max_age) failures.append("stale_result");
    out["failed_gates"]=failures;
    out["accepted"]=failures.empty();
    out["reason"]=failures.empty() ? Json::Value("accepted") : failures[0];
  }
public:
  Json::Value handle(const Json::Value& req, Clock::time_point started) {
    Json::Value out=defaults(generation_);
    const auto cpu=cpu_ms();
    try {
      require(req.isObject(),"request_object_required");
      const auto op=field(req,"op");
      if (op=="load") load(req,out);
      else if (op=="match") match(req,out,started);
      else throw std::runtime_error("unknown_op");
    } catch(const std::exception& e) {
      const std::string reason=e.what();
      out["ok"]=reason=="insufficient_submap" || reason=="submap_point_limit";
      out["accepted"]=false;
      out["reason"]=reason;
    }
    out["cpu_ms"]=cpu_ms()-cpu;
    out["peak_rss_kib_process"]=Json::Int64(maxrss());
    out["elapsed_ms"]=milliseconds(started);
    if (out["accepted"].asBool() && out["elapsed_ms"].asDouble()>out["budget_ms"].asDouble()) {
      out["accepted"]=false; out["reason"]="time_budget"; out["failed_gates"].append("time_budget");
    }
    return out;
  }
  Json::UInt64 generation() const {return generation_;}
};
}  // namespace
}  // namespace culvert

#include "session.hpp"
namespace culvert {
struct Session::Impl { SessionState server; };
Session::Session() : impl_(std::make_unique<Impl>()) {}
Session::~Session() = default;
Json::Value Session::handle(const Json::Value& request, Clock::time_point start) {
  return impl_->server.handle(request,start);
}
Json::Value Session::invalid_json(Clock::time_point start) const {
  auto out=defaults(impl_->server.generation());
  out["reason"]="invalid_json_or_size";
  out["elapsed_ms"]=milliseconds(start);
  return out;
}
}  // namespace culvert
