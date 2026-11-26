#pragma once

#include "registration.hpp"
#include "session.hpp"
#include <pcl/pcl_config.h>
#include <algorithm>

namespace culvert {

// Checked client for synchronous callers. The owned worker is deliberately private:
// a successful load and its map generation cannot be changed outside this binding.
class CheckedSession {
public:
  Json::Value handle(const Json::Value& request, Clock::time_point received=Clock::now()) {
    const bool match=request.isObject() && request["op"]=="match";
    Json::Value result;
    if (match) {
      const auto error=binding_error(request);
      if (!error.empty()) {
        result["ok"]=false;
        result["accepted"]=false;
        result["converged"]=false;
        result["reason"]=error;
        result["generation"]=generation_;
        result["requested_generation"]=request["generation"];
        result["backend"]=request["backend"];
        result["scan_id"]=request["scan_id"];
        result["stamp_ns"]=request["stamp_ns"];
        result["source_frame"]=request["source_frame"];
        result["elapsed_ms"]=milliseconds(received);
        if (!metadata_.isNull()) {
          result["map_id"]=metadata_["id"];
          result["map_version"]=metadata_["version"];
          result["map_sha256"]=metadata_["sha256"];
          result["target_frame"]=metadata_["frame"];
        }
        return describe(std::move(result));
      }
    }
    result=worker_.handle(request,received);
    if (match && !result.isMember("backend")) result["backend"]=request["backend"];
    if (request.isObject() && request["op"]=="load" && result["ok"].asBool()) {
      metadata_=result["map"];
      generation_=result["generation"].asUInt64();
    }
    return describe(std::move(result));
  }

private:
  Session worker_;
  Json::Value metadata_;
  Json::UInt64 generation_=0;

  std::string binding_error(const Json::Value& request) const {
    if (metadata_.isNull()) return "map_unloaded";
    for (const auto* name : {"map_id","version","sha256","target_frame"}) {
      if (!request[name].isString() || request[name].asString().empty())
        return std::string("missing_or_invalid_")+name;
    }
    const auto& g=request["generation"];
    if ((g.type()!=Json::intValue && g.type()!=Json::uintValue) || !g.isUInt64())
      return "missing_or_invalid_generation";
    if (g.asUInt64()!=generation_) return "stale_generation";
    if (request["map_id"]!=metadata_["id"]) return "map_id_mismatch";
    if (request["version"]!=metadata_["version"]) return "map_version_mismatch";
    if (request["sha256"]!=metadata_["sha256"]) return "map_sha256_mismatch";
    if (request["target_frame"]!=metadata_["frame"]) return "target_frame_mismatch";
    return {};
  }

  static bool verified_pose(const Json::Value& rows) {
    if (!rows.isArray() || rows.size()!=4) return false;
    Pose p;
    for (int i=0; i<4; ++i) {
      if (!rows[i].isArray() || rows[i].size()!=4) return false;
      for (int j=0; j<4; ++j) {
        if (!rows[i][j].isNumeric()) return false;
        p(i,j)=rows[i][j].asDouble();
      }
    }
    return valid_pose(p,1e-3);
  }

  static Json::Value version(const Json::Value& backend) {
    if (backend=="icp") return std::string("pcl-")+PCL_VERSION_PRETTY+"/legacy-two-stage";
    if (backend=="gicp") return "small_gicp@fa0cfc983417c044b6036f71dfa448038bb9c820/gicp";
    return Json::Value();
  }

  static Json::Value submap_id(const Json::Value& r) {
    const auto& sub=r["submap"];
    if (!sub["cache_center_m"].isArray()) return Json::Value();
    Json::Value tuple(Json::arrayValue);
    for (const auto* k : {"map_id","map_version","map_sha256","target_frame","generation"})
      tuple.append(r[k]);
    for (const auto* k : {"cache_center_m","grid_m","radius_m","effective_radius_m"})
      tuple.append(sub[k]);
    Json::StreamWriterBuilder writer;
    writer["indentation"]="";
    return Json::writeString(writer,tuple);
  }

  static Json::Value describe(Json::Value r) {
    const Json::Value& raw=r;
    const auto& stages=raw["stages"];
    const bool solved=stages["rough"]["T_target_source"].isArray() || stages["gicp"]["iterations_native"].isUInt64();
    const bool valid=solved && verified_pose(raw["T_target_source"]) && verified_pose(raw["T_map_W"]);
    const bool completed=solved && raw["quality"].isObject() && raw["failed_gates"].isArray();
    r["backend_version"]=version(raw["backend"]);
    r["computation_completed"]=completed;
    r["valid_se3"]=valid;
    r["geometry_gate_passed"]=Json::nullValue;
    r["prior_gate_passed"]=Json::nullValue;
    r["submap_id"]=submap_id(r);
    r["candidate"]=Json::nullValue;
    r["absolute_truth_available"]=false;
    r["absolute_pose_error"]=Json::nullValue;
    if (completed) {
      const auto& failures=raw["failed_gates"];
      auto failed=[&](const char* name){ return std::find(failures.begin(),failures.end(),Json::Value(name))!=failures.end(); };
      r["geometry_gate_passed"]=!(failed("low_overlap") || failed("high_rmse") || failed("low_coverage") ||
                                  failed("weak_geometry") || failed("weak_se3"));
      r["prior_gate_passed"]=!failed("excessive_correction");
    }
    if (valid) {
      auto& candidate=r["candidate"];
      candidate["T_target_source"]=raw["T_target_source"];
      candidate["T_map_W"]=raw["T_map_W"];
      candidate["accepted"]=raw["accepted"].asBool() && completed;
    } else {
      r["T_target_source"]=Json::nullValue;
      r["T_map_W"]=Json::nullValue;
    }
    if (r["accepted"].asBool() && (!valid || !completed)) {
      r["accepted"]=false;
      r["reason"]="unverified_candidate";
    }
    return r;
  }
};
}  // namespace culvert
