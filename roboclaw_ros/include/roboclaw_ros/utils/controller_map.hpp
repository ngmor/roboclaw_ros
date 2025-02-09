/// @file
/// @brief Utilities for reading controller configuration from ROS parameter files

# pragma once

#include <string>
#include <unordered_map>
#include <memory>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <roboclaw/roboclaw.hpp>

namespace roboclaw
{

namespace utils
{

/// @brief struct containing configuration information about a controller
struct ControllerInfo
{

  /// @brief type of the map which relates semantic motor name to channel
  using motor_map_t = std::unordered_map<std::string, MotorSelect>;

  /// @brief semantic controller name
  std::string name;

  /// @brief map between semantic motor names and channels
  motor_map_t motors;

  /// @brief roboclaw driver controller config (address, encoder counts per rev)
  std::shared_ptr<ControllerConfig> config = nullptr;

  /// @brief get the address of the controller
  ControllerConfig::address_t address() const;

  /// @brief get the encoder counts per revolution based on motor channel
  /// @param motor motor channel
  /// @return encoder counts per rev of the specified channel
  enc_count_t enc_counts_per_rev(MotorSelect motor) const;

  /// @brief get the encoder counts per revolution based on semantic motor name
  /// @param motor semantic motor name
  /// @return encoder counts per rev of the specified motor
  enc_count_t enc_counts_per_rev(const std::string& motor) const;

  /// @brief output ControllerInfo struct to stream
  friend std::ostream& operator<<(std::ostream& os, const ControllerInfo& controller);
};


/// @brief A helper class which constructs an unordered map of ControllerInfo,
/// with the semantic controller name as the key to the map. The class does this by
/// looking for parameters on an input node. In order for this to work, these node
/// options must be used in the rclcpp::Node constructor:
/// @code{.cpp}
/// rclcpp::NodeOptions()
///   .allow_undeclared_parameters(true)
///   .automatically_declare_parameters_from_overrides(true)
/// @endcode
/// Then, a yaml parameter file can be used to specify a controller list as node parameters
/// formatted like so:
/// @code{.yaml}
/// controllers:
///   drive:
///     address: 0x80
///     M1:
///       name: right
///       enc_counts_per_rev: 6400
///     M2:
///       name: left
///       enc_counts_per_rev: 6400
///   arms:
///     address: 0x81
///     M1:
///       name: right
///       enc_counts_per_rev: 9600
///     M2:
///       name: left
///       enc_counts_per_rev: 9600
///   head:
///     address: 0x82
///     M2:
///       name: tilt
///       enc_counts_per_rev: 3200
/// @endcode
class ControllerMap
{
public:

  /// @brief underlying unordered map type
  using map_t = std::unordered_map<std::string, ControllerInfo>;

  /// @brief Construct the controller map
  /// @param node node which has the "controllers" parameter that can be searched for controllers
  ControllerMap(const rclcpp::Node& node);

  /// @brief const iterator to the beggining of the map
  map_t::const_iterator begin() const { return controllers_.cbegin(); }

  /// @brief const iterator to the end of the map
  map_t::const_iterator end() const { return controllers_.cend(); }

  /// @brief check if the map is empty
  bool empty() const { return controllers_.empty(); }

  /// @brief get the size of the map
  size_t size() const { return controllers_.size(); }

  /// @brief get controller info by semantic controller name
  const ControllerInfo& at(const std::string& controller) const
  {
    return controllers_.at(controller);
  }

  /// @brief get the iterator of the map member with the specified controller name
  /// If it doesn't exist, will return the end() iterator
  map_t::const_iterator find(const std::string& controller) const
  {
    return controllers_.find(controller);
  }

  /// @brief check if the map contains a certain controller name
  bool contains(const std::string& controller) const
  {
    return controllers_.find(controller) != controllers_.end();
  }

  /// @brief output ControllerMap to stream
  friend std::ostream& operator<<(std::ostream& os, const ControllerMap& controllers);

private:

  /// @brief internal map of controllers
  map_t controllers_;
};

}  // namespace utils

}  // namespace roboclaw