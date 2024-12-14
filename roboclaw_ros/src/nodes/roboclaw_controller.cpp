/// @file
/// @brief Node to control RoboClaw motor controllers over a serial port

#include <unordered_map>
#include <stdexcept>

#include <roboclaw_ros/utils/controller_map.hpp>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/empty.hpp>

using roboclaw::utils::ControllerMap;

class RoboClawControllerNode : public rclcpp::Node
{
private:

  using pub_motor_feedback_map_t =
    std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr>; // TODO change
  using sub_motor_velocity_map_t =
    std::unordered_map<std::string, rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr>; // TODO change

  // CLASS MEMBERS --------------------------------------------------------------------------------

  const ControllerMap controller_map_ {*this};

  std::string device_;

  std::unordered_map<std::string, pub_motor_feedback_map_t> pub_feedback_;
  std::unordered_map<std::string, sub_motor_velocity_map_t> sub_velocity_;

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

    if (!has_parameter("device"))
    {
      throw std::runtime_error{"No serial 'device' parameter provided, aborting"};
    }

    device_ = get_parameter("device").as_string();

    RCLCPP_INFO_STREAM(get_logger(), controller_map_);

    // TOPICS --------------------------------------------------------------------------------
    for (const auto& controller : controller_map_)
    {
      const auto& controller_name = controller.first;
      const auto& controller_info = controller.second;

      // create maps for this controller
      sub_velocity_[controller_name] = {};
      pub_feedback_[controller_name] = {};

      for (const auto& motor : controller_info.motors)
      {
        const auto& motor_name = motor.first;

        const auto topic_namespace = controller_name + "/" + motor_name + "/";

        // Create a velocity setpoint callback using a lambda function which calls
        // a class method (passing in the name of the controller/motor as well)
        sub_velocity_[controller_name][motor_name] = create_subscription<std_msgs::msg::Empty>(
          topic_namespace + "velocity_setpoint",
          rclcpp::QoS{10},
          [this, controller_name, motor_name](const std_msgs::msg::Empty& msg)
          {
            sub_velocity_callback(msg, controller_name, motor_name);
          }
        );

        // Create publishers for velocity and position feedback
        // TODO publish to these in a timer loop
        pub_feedback_[controller_name][motor_name] = create_publisher<std_msgs::msg::Empty>(
          topic_namespace + "feedback",
          rclcpp::QoS{10}
        );
      }

      RCLCPP_INFO_STREAM(get_logger(), get_fully_qualified_name() << " node started");
    }

  }

private:

  void sub_velocity_callback(
    const std_msgs::msg::Empty&,
    const std::string& controller,
    const std::string& motor
  )
  {
    // TODO implement
    std::cout << controller << " " << motor << std::endl;
  }

  // void pub_feedback_callback(){

  // }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoboClawControllerNode>());
  rclcpp::shutdown();
  return 0;
}