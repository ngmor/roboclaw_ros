/// @file
/// @brief Node to control RoboClaw motor controllers over a serial port

#include <rclcpp/rclcpp.hpp>

#include <roboclaw_ros/utils/controller_map.hpp>

using roboclaw::utils::ControllerMap;

class RoboClawControllerNode : public rclcpp::Node
{
public:

  /// @brief initialize the node
  RoboClawControllerNode()
  : Node(
    "roboclaw_controller",
    // Automatically declare parameters to read controller map
    rclcpp::NodeOptions()
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true)
  )
  {
    // PARAMETERS --------------------------------------------------------------------------------

    RCLCPP_INFO_STREAM(get_logger(), controller_map_);

    RCLCPP_INFO_STREAM(get_logger(), get_fully_qualified_name() << " node started");
  }
private:

  const ControllerMap controller_map_ {*this};

};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoboClawControllerNode>());
  rclcpp::shutdown();
  return 0;
}