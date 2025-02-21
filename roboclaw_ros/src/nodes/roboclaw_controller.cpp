/// @file
/// @brief Node to control RoboClaw motor controllers over a serial port

#include <sstream>
#include <unordered_map>
#include <stdexcept>

#include <roboclaw_ros/utils/controller_map.hpp>
#include <roboclaw_ros/utils/parameter.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <roboclaw_interfaces/msg/motor_feedback.hpp>
#include <roboclaw_interfaces/msg/velocity_setpoint.hpp>

#include <roboclaw/roboclaw.hpp>


using roboclaw::utils::ControllerMap;
using roboclaw::utils::declare_and_get_parameter;
using roboclaw::RoboClaw;
using roboclaw::ReturnCode;

class RoboClawControllerNode : public rclcpp::Node
{
private:

  /// @brief Motor-related data used/modified by the node
  struct MotorData
  {

    /// @brief Publisher for the motor's feedback data
    rclcpp::Publisher<roboclaw_interfaces::msg::MotorFeedback>::SharedPtr pub_feedback = nullptr;

    /// @brief Subscriber for motor's velocity setpoint
    rclcpp::Subscription<roboclaw_interfaces::msg::VelocitySetpoint>::SharedPtr sub_velocity = nullptr;

    /// @brief Time the motor recieved the last command
    rclcpp::Time last_command_time {0};

    /// @brief Flag to indicate if the motor has already timed out
    bool timed_out = false;
  };

  // CLASS MEMBERS --------------------------------------------------------------------------------
  rclcpp::TimerBase::SharedPtr tmr_pub_feedback_;
  rclcpp::TimerBase::SharedPtr tmr_evaluate_timeout_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_open_port_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_close_port_;

  /// @brief Controller info map
  const ControllerMap controller_map_ {*this};
  
  /// @brief Map of controllers -> motors -> motor data
  std::unordered_map<std::string, std::unordered_map<std::string, MotorData>> motor_data_;

  std::string device_ = "";
  RoboClaw driver_;
  int baudrate_ = 0;
  double velocity_deadband_ = 0.0;
  rclcpp::Duration command_timeout_ {std::chrono::duration<double>(0)};

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

    if (!has_parameter("baudrate"))
    {
      throw std::runtime_error{"No serial 'baudrate' parameter provided, aborting"};
    }

    baudrate_ = get_parameter("baudrate").as_int();

    velocity_deadband_ = std::abs(declare_and_get_parameter<double>(
      *this,
      "velocity_deadband",
      "If velocity setpoint magnitude is less than this value (rad/s) then the duty cycle is set to 0",
      1.0e-3
    ));

    command_timeout_ = rclcpp::Duration{std::chrono::duration<double>(declare_and_get_parameter<double>(
      *this,
      "command_timeout",
      "If the controller hasn't received a new command for a motor for this duration (sec) then it will timeout and stop",
      2.0
    ))};

    const auto feedback_rate = std::abs(declare_and_get_parameter<double>(
      *this,
      "feedback_rate",
      "Rate (Hz) at which feedback for each motor channel is published",
      100.0
    ));

    const auto timeout_evaluation_rate = std::abs(declare_and_get_parameter<double>(
      *this,
      "timeout_evaluation_rate",
      "Rate (Hz) at which timeouts are evaluated",
      10
    ));


    RCLCPP_INFO_STREAM(get_logger(), controller_map_);

    // TOPICS --------------------------------------------------------------------------------
    for (const auto& controller : controller_map_)
    {
      const auto& controller_name = controller.first;
      const auto& controller_info = controller.second;

      // create motor data map for this controller
      motor_data_[controller_name] = {};

      for (const auto& motor : controller_info.motors)
      {
        const auto& motor_name = motor.first;

        // Create motor data map for this motor
        motor_data_[controller_name][motor_name] = {};
        auto& this_motor_data = motor_data_[controller_name][motor_name];

        const auto topic_namespace = controller_name + "/" + motor_name + "/";

        // initialize the controller time to the current time
        this_motor_data.last_command_time = get_clock()->now();
        this_motor_data.timed_out = false;

        // Create a velocity setpoint callback using a lambda function which calls
        // a class method (passing in the name of the controller/motor as well)
        this_motor_data.sub_velocity = create_subscription<roboclaw_interfaces::msg::VelocitySetpoint>(
          topic_namespace + "velocity_setpoint",
          rclcpp::QoS{10},
          [this, controller_name, motor_name](const roboclaw_interfaces::msg::VelocitySetpoint& msg)
          {
            sub_velocity_callback(msg, controller_name, motor_name);
          }
        );

        // Create publishers for velocity and position feedback
        this_motor_data.pub_feedback = create_publisher<roboclaw_interfaces::msg::MotorFeedback>(
          topic_namespace + "feedback",
          rclcpp::QoS{10}
        );
      }
    }

    // TIMER --------------------------------------------------------------------------------
    tmr_pub_feedback_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / feedback_rate),
      std::bind(&RoboClawControllerNode::tmr_pub_feedback_callback, this)
    );

    tmr_evaluate_timeout_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / timeout_evaluation_rate),
      std::bind(&RoboClawControllerNode::tmr_evaluate_timeout_callback, this)
    );

    // SERVICES -----------------------------------------------------------------------------
    srv_open_port_ = create_service<std_srvs::srv::Trigger>(
      "open_port",
      std::bind(
        &RoboClawControllerNode::srv_open_port_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );

    srv_close_port_ = create_service<std_srvs::srv::Trigger>(
      "close_port",
      std::bind(
        &RoboClawControllerNode::srv_close_port_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );

    // Open driver
    ReturnCode ret = driver_.open(device_, baudrate_);

    if(ret == ReturnCode::OK)
    {
      RCLCPP_INFO_STREAM(get_logger(),
        "Port " << device_ << " opened successfully at baudrate " << baudrate_
      );
    }
    else
    {
      RCLCPP_ERROR_STREAM(get_logger(),
        "Unable to initialize RoboClaw at port " << device_
        << ", baudrate " << baudrate_
        << " with error: " << static_cast<int>(ret)
        << "\nCall open_port service to retry."
      );
    }

    RCLCPP_INFO_STREAM(get_logger(), get_fully_qualified_name() << " node started");
  }

  /// @brief destructor for roboclaw controller node, stops all motors on exit
  ~RoboClawControllerNode()
  {
    stop_all_motors();
  }

private:
  /// @brief stop a given motor
  /// @param cfg the controller configuration
  /// @param motor the specific motor to turn off
  /// @return error code
  ReturnCode stop_motor(const roboclaw::ControllerConfig& cfg, roboclaw::MotorSelect motor)
  {
    return driver_.set_duty(cfg, motor, 0.0);
  }

  /// @brief stop all of the motors
  void stop_all_motors()
  {
    // go through all controllers
    for (const auto& controller : controller_map_)
    {
      const auto& controller_info = controller.second;
      for (const auto& motor : controller_info.motors)
      {
        // stop each motor
        const auto motor_channel = motor.second;
        stop_motor(*controller_info.config, motor_channel);
      }
    }
  }

  /// @brief service to open the RoboClaw driver serial port
  void srv_open_port_callback(
    std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response
  )
  {
    if (driver_.is_open())
    {
      response->message = "Serial port is already open.";
      RCLCPP_WARN_STREAM(get_logger(), response->message);
      response->success = true;
      return;
    }

    ReturnCode ret = driver_.open(device_, baudrate_);

    if (ret == ReturnCode::OK)
    {
      std::stringstream ss;
      ss << "Port " << device_ << " opened successfully with baudrate " << baudrate_;
      response->message = ss.str();
      RCLCPP_INFO_STREAM(get_logger(), response->message);
      response->success = true;
    }
    else
    {
      std::stringstream ss;
      ss
        << "Unable to initialize RoboClaw at port " << device_
        << ", baudrate " << baudrate_
        << " with error: " << static_cast<int>(ret)
        << "\nCall open_port service to retry."
      ;
      response->message = ss.str();
      RCLCPP_ERROR_STREAM(get_logger(), response->message);
      response->success = false;
    }
  }

  /// @brief service to close the RoboClaw driver serial port
  void srv_close_port_callback(
    std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response
  )
  {
    if (!driver_.is_open())
    {
      response->message = "Serial port is already closed.";
      RCLCPP_WARN_STREAM(get_logger(), response->message);
      response->success = true;
      return;
    }

    ReturnCode ret = driver_.close();

    if (ret == ReturnCode::OK)
    {
      std::stringstream ss;
      ss << "Port " << device_ << " closed successfully";
      response->message = ss.str();
      RCLCPP_INFO_STREAM(get_logger(), response->message);
      response->success = true;
    }
    else
    {
      std::stringstream ss;
      ss << "Error closing port " << device_ << ": " << static_cast<int>(ret);
      response->message = ss.str();
      RCLCPP_ERROR_STREAM(get_logger(), response->message);
      response->success = false;
    }
  }

  /// @brief set the specified motor on the specified controller to the
  /// specified velocity
  /// @param velocity velocity setpoint message
  /// @param controller the controller to use
  /// @param motor the intended motor
  void sub_velocity_callback(
    const roboclaw_interfaces::msg::VelocitySetpoint& msg,
    const std::string& controller,
    const std::string& motor
  )
  {
    // look for the passed controller in the controller map
    const auto target_controller_itr = controller_map_.find(controller);
    if (target_controller_itr != controller_map_.end())
    {
      const auto& target_controller_info = target_controller_itr->second;
      // look for the passed motor in the controller
      auto target_motor_itr = target_controller_info.motors.find(motor);
      if (target_motor_itr != target_controller_info.motors.end())
      {
        const auto target_motor_channel = target_motor_itr->second;

        // check if the velocity is set to near 0
        if(std::abs(msg.velocity) < velocity_deadband_)
        {
          stop_motor(*target_controller_info.config, target_motor_channel);
        }
        else
        {
          auto ret = driver_.set_velocity(*target_controller_info.config, target_motor_channel, msg.velocity);

          // check that the velocity was set
          if (ret != ReturnCode::OK)
          {
            RCLCPP_ERROR_STREAM(get_logger(), "Unable to set velocity for " << controller << "." << motor << " with error: " << static_cast<int>(ret));
          }
        }

        // reset the message timer for this motor
        motor_data_[controller][motor].last_command_time = get_clock()->now();
        motor_data_[controller][motor].timed_out = false;
      }
      else
      {
        RCLCPP_ERROR_STREAM(get_logger(), controller << "." << motor << " not found");
      }
    }
    else
    {
      RCLCPP_ERROR_STREAM(get_logger(), controller << "." << motor << " not found");
    }
  }

  /// @brief timer callback to get the publish motor feedback
  void tmr_pub_feedback_callback()
  {
    if (!driver_.is_open())
    {
      // Publish invalid feedback for all motors
      for (const auto& controller : controller_map_)
      {
        const auto& controller_name = controller.first;
        const auto& controller_info = controller.second;
        for (const auto& motor : controller_info.motors)
        {
          const auto& motor_name = motor.first;
          roboclaw_interfaces::msg::MotorFeedback motor_feedback;
          motor_feedback.valid_positon = false;
          motor_feedback.valid_velocity = false;
          motor_feedback.stamp = get_clock()->now();
          motor_data_[controller_name][motor_name].pub_feedback->publish(motor_feedback);
        }
      }
      return;
    }

    for (const auto& controller : controller_map_)
    {
      const auto& controller_name = controller.first;
      const auto& controller_info = controller.second;
      const auto [ret_pos, positions] = driver_.get_enc_radians(*controller_info.config);

      // check that position was retreived
      if (ret_pos != ReturnCode::OK)
      {
        RCLCPP_ERROR_STREAM(get_logger(), 
          "Unable to get position using controller " << controller_name
          << ", error: " << static_cast<int>(ret_pos)
        );
      }

      for (const auto& motor : controller_info.motors)
      {
        roboclaw_interfaces::msg::MotorFeedback motor_feedback;

        const auto& motor_name = motor.first;
        const auto motor_channel = motor.second;
        const auto [ret_vel, velocity] = driver_.get_velocity_radians(*controller_info.config, motor_channel);

        motor_feedback.velocity = velocity;
        motor_feedback.valid_velocity = ret_vel == ReturnCode::OK;

        // check that velocity was retreived
        if (ret_vel != ReturnCode::OK)
        {
          RCLCPP_ERROR_STREAM(get_logger(),
            "Unable to get velocity from " << controller_name << "." << motor_name
            << ", error: " << static_cast<int>(ret_vel)
          );
        }

        motor_feedback.position = positions[to_index(motor_channel)];
        motor_feedback.valid_positon = ret_pos == ReturnCode::OK;
        motor_feedback.stamp = get_clock()->now();
        // publish motor feedback with the current position and velocity
        motor_data_[controller_name][motor_name].pub_feedback->publish(motor_feedback);
      }
    }
  }

  /// @brief check if any of the motors have timed out
  void tmr_evaluate_timeout_callback()
  {
    if (!driver_.is_open())
    {
      return;
    }

    for (const auto& controller : controller_map_)
    {
      const auto& controller_name = controller.first;
      const auto& controller_info = controller.second;
      for (const auto& motor : controller_info.motors)
      {
        const auto& motor_name = motor.first;
        const auto motor_channel = motor.second;
        auto& motor_data = motor_data_[controller_name][motor_name];

        // If already timed out, skip evaluation
        if (motor_data.timed_out)
        {
          continue;
        }

        if ((get_clock()->now() - motor_data.last_command_time) > command_timeout_)
        {
          stop_motor(*controller_info.config, motor_channel);
          motor_data.timed_out = true;
        }
      }
    }
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoboClawControllerNode>());
  rclcpp::shutdown();
  return 0;
}