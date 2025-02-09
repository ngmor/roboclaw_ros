/// @file
/// @brief Node to control RoboClaw motor controllers over a serial port

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

    /// @brief Time the motor revieved the last command
    rclcpp::Time last_command_time {0};
  };

  // TODO make service to open driver

  // TODO make service to close driver

  // CLASS MEMBERS --------------------------------------------------------------------------------
  rclcpp::TimerBase::SharedPtr tmr_pub_feedback_;
  rclcpp::TimerBase::SharedPtr tmr_evaluate_timeout_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_open_driver_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_close_driver_;

  /// @brief Controller info map
  const ControllerMap controller_map_ {*this};
  
  /// @brief Map of controllers -> motors -> motor data
  std::unordered_map<std::string, std::unordered_map<std::string, MotorData>> motor_data_;

  std::string device_ = "";
  RoboClaw driver_;
  int baudrate_ = 0;
  double velocity_deadband_ = 0.0;
  int pub_feedback_freq_ = 0;
  int check_messages_freq_ = 0;
  rclcpp::Duration command_timeout_ {std::chrono::duration<double>(0)};
  rclcpp::Logger logger_ = get_logger();

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
      "velocity.deadband",
      "If velocity setpoint magnitude is less than this value then the duty cylce is set to 0",
      1.0e-3
    ));

    command_timeout_ = rclcpp::Duration{std::chrono::duration<double>(declare_and_get_parameter<double>(
      *this,
      "command.timeout",
      "If the controller hasn't received a new command for a motor for this many seconds then it will timeout and stop",
      2.0
    ))};

    pub_feedback_freq_ = std::abs(declare_and_get_parameter<int>(
      *this,
      "frequency.feedback",
      "Set the frequency of the pub_feedback_timer_callback callback",
      100
    ));

    check_messages_freq_ = std::abs(declare_and_get_parameter<int>(
      *this,
      "frequency.messages",
      "Set the frequency of the evaluate_timeout_callback callback",
      100
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
        motor_data_[controller_name][motor_name].last_command_time = get_clock()->now();

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

      RCLCPP_INFO_STREAM(get_logger(), get_fully_qualified_name() << " node started");
    }

    // TIMER --------------------------------------------------------------------------------
    tmr_pub_feedback_ = create_wall_timer(
        static_cast<std::chrono::microseconds>(static_cast<int>(100)),
        std::bind(&RoboClawControllerNode::pub_feedback_timer_callback, this)
      );

    tmr_evaluate_timeout_ = create_wall_timer(
        static_cast<std::chrono::microseconds>(static_cast<int>(100)),
        std::bind(&RoboClawControllerNode::evaluate_timeout_callback, this)
      );

    // SERVICES -----------------------------------------------------------------------------
    srv_open_driver_ = create_service<std_srvs::srv::Trigger>(
      "/open_driver",
      std::bind(&RoboClawControllerNode::srv_open_driver, this, std::placeholders::_1, std::placeholders::_2)
    );

    srv_close_driver_ = create_service<std_srvs::srv::Trigger>(
      "/close_driver",
      std::bind(&RoboClawControllerNode::srv_close_driver, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Open driver
    ReturnCode ret = driver_.open(device_, baudrate_);

    rclcpp::Logger logger_ = get_logger();
    //quit if initialization fails
    if(ret != ReturnCode::OK)
    {
      RCLCPP_ERROR_STREAM(logger_, "unable to initialize roboclaw with error: " << static_cast<int>(ret));
      // retry the connection to the roboclaw
      int count_retry = 0;
      while (count_retry < 10 && ret != ReturnCode::OK){
        RCLCPP_INFO_STREAM(logger_,"retrying connection...");
        ret = driver_.open(device_, baudrate_);
        count_retry++;
      }
      if(ret != ReturnCode::OK){
        RCLCPP_ERROR_STREAM(logger_, "unable to initialize roboclaw with error: " << static_cast<int>(ret) << " after 10 retries. \n Call open_driver service to retry.");
      }
    }
  }

  /// @brief destructor for roboclaw controller node
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

  /// @brief service to open the driver
  /// @param empty
  /// @param empty
  void srv_open_driver(
    std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr)
  {
    ReturnCode ret = driver_.open(device_, baudrate_);
        if (ret != ReturnCode::OK){
      RCLCPP_ERROR_STREAM(logger_, "unable to open port with error: " << static_cast<int>(ret) << " \n To retry use open_driver service.");
    }
    else{
      RCLCPP_INFO_STREAM(logger_,"Port opened.");
    }
  }

  /// @brief service to close the driver
  /// @param empty
  /// @param empty
  void srv_close_driver(
    std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr)
  {
    ReturnCode ret = driver_.close();
    if (ret != ReturnCode::OK){
      RCLCPP_ERROR_STREAM(logger_, "unable to close port with error: " << static_cast<int>(ret) << " \n To retry use close_driver service.");
    }
    else{
      RCLCPP_INFO_STREAM(logger_,"Port closed.");
    }
  }

  /// @brief subscribes to the velocity setpoints
  /// @param velocity the new setpoint
  /// @param controller the controller to use
  /// @param motor the intended motor
  void sub_velocity_callback(
    const roboclaw_interfaces::msg::VelocitySetpoint& velocity,
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
        // Note this is not a reference since it is a cheaply copyable roboclaw::MotorSelect (uint8_t)
        const auto target_motor_channel = target_motor_itr->second;
        // check if the velocity is set to 0
        if(std::abs(velocity.velocity) < velocity_deadband_)
        {
          stop_motor(*target_controller_info.config, target_motor_channel);
        }
        else
        {
          auto ret = driver_.set_velocity(*target_controller_info.config, target_motor_channel, velocity.velocity);
          // check that the velocity was set
          if (ret != ReturnCode::OK)
          {
            RCLCPP_ERROR_STREAM(logger_, "unable to set velocity for motor " << motor << " using controller " << controller << " with error: " << static_cast<int>(ret));
          }
          // reset the message timer for this motor
          motor_data_[controller][motor].last_command_time = this->get_clock()->now();
        }
      }
      else
      {
        std::cerr << "motor not found" << std::endl;
      }
    }
    else
    {
      std::cerr << "controller not found" << std::endl;
    }
  }

  /// @brief timer callback to get the state of the motors
  void pub_feedback_timer_callback(){
    for (const auto& controller : controller_map_)
    {
      const auto& controller_name = controller.first;
      const auto& controller_info = controller.second;
      const auto [ret_pos, positions] = driver_.get_enc_radians(*controller_info.config);

      // check that position was retreived
      if (ret_pos != ReturnCode::OK)
      {
        RCLCPP_ERROR_STREAM(logger_, "unable to get position using controller " << controller_name);
      }

      for (const auto& motor : controller_info.motors)
      {
        roboclaw_interfaces::msg::MotorFeedback motor_feedback;

        const auto& motor_name = motor.first;
        // Note this is not a reference since it is a cheaply copyable roboclaw::MotorSelect (uint8_t)
        const auto& motor_channel = motor.second;
        const auto [ret_vel, velocity] = driver_.get_velocity_radians(*controller_info.config, motor_channel);

        // check that velocity was retreived
        if (ret_vel != ReturnCode::OK)
        {
          RCLCPP_ERROR_STREAM(logger_, "unable to get velocity from motor " << motor_name << " using controller " << controller_name);
        }
        else{
          motor_feedback.velocity = velocity;
          motor_feedback.valid_velocity = 1;
        }

        motor_feedback.position = positions[to_index(motor_channel)]; 

        // publish motor feedback with the current position and velocity
        motor_data_[controller_name][motor_name].pub_feedback->publish(motor_feedback);
      }
    }
  }

  /// @brief check if any of the motors have timed out
  void evaluate_timeout_callback(){
    for (const auto& controller : controller_map_)
    {
      const auto& controller_info = controller.second;
      for (const auto& motor : controller_info.motors)
      {
        rclcpp::Time last_time = motor_data_[controller.first][motor.first].last_command_time;
        rclcpp::Time current_time = get_clock()->now();
        
        if ((get_clock()->now() - motor_data_[controller.first][motor.first].last_command_time) > command_timeout_){
          stop_motor(*controller_info.config, motor.second);
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