#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sstream>
#include <stdexcept>

namespace startup_parameters {

inline rcl_interfaces::msg::ParameterDescriptor readOnly()
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  return descriptor;
}

// Inspect the original ROS overrides to distinguish "explicit default" from absence.
// Declare the canonical key with the effective value even when just the old key is supplied.
template <typename T>
T declareWithAlias(rclcpp::Node &node, const char *name, const char *legacy, T fallback)
{
  const auto &overrides = node.get_node_parameters_interface()->get_parameter_overrides();
  const auto descriptor = readOnly();
  const bool has_legacy = overrides.count(legacy) != 0;
  const T old_value = has_legacy ? node.declare_parameter<T>(legacy, fallback, descriptor) : fallback;
  const T value = node.declare_parameter<T>(name, old_value, descriptor);
  if (has_legacy) {
    if (overrides.count(name) && value != old_value) {
      std::ostringstream message;
      message << std::boolalpha << "Conflicting explicit parameters: "
              << name << '=' << value << ", " << legacy << '=' << old_value;
      throw std::invalid_argument(message.str());
    }
    RCLCPP_WARN(node.get_logger(), "Deprecated parameter %s; use %s", legacy, name);
  }
  return value;
}

}  // namespace startup_parameters
