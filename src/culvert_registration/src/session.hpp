#pragma once

#include <chrono>
#include <json/json.h>
#include <memory>

namespace culvert {
using Clock = std::chrono::steady_clock;

// Owns one map generation and its bounded caches. Calls are serialized by the owner.
// The integration boundary has no ROS, TF, node, or transport dependency.
class Session {
public:
  Session();
  ~Session();
  Json::Value handle(const Json::Value& request, Clock::time_point received = Clock::now());
  Json::Value invalid_json(Clock::time_point received) const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace culvert
