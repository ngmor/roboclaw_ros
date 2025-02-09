/// @file
/// @brief Utilities for working ROS parameters

#pragma once

#include <rclcpp/rclcpp.hpp>

namespace roboclaw
{

namespace utils
{

/// @brief declare and return the value of a parameter. Handles if the parameter has already been
/// declared (for example, via overrides)
/// @tparam T type of the parameter being declared
/// @param node pointer to the node using the parameter
/// @param name name of the parameter
/// @param description explanation of what the parameter is
/// @param default_value the value this parameter defaults to if not provided
/// @return the value of the parameters
template<typename T>
T declare_and_get_parameter(rclcpp::Node& node, const std::string& name, const std::string& description, const T& default_value)
{
  T override_value {};
  bool was_declared_from_override = false;

  if(node.has_parameter(name))
  {
    override_value = node.get_parameter(name).get_value<T>();
    was_declared_from_override = true;
    node.undeclare_parameter(name);
  }

  const auto descriptor = rcl_interfaces::msg::ParameterDescriptor{}.set__description(description);
  node.declare_parameter(name, default_value, descriptor);

  if (was_declared_from_override)
  {
    const auto param = rclcpp::Parameter{name, override_value};
    node.set_parameter(param);
  }

  return node.get_parameter(name).get_value<T>();
}

}  // namespace utils

}  // namespace roboclaw