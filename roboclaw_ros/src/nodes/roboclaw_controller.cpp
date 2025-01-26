/// @file
/// @brief Node to control RoboClaw motor controllers over a serial port

#include <unordered_map>
#include <stdexcept>

#include <roboclaw_ros/utils/controller_map.hpp>
#include <roboclaw_ros/utils/parameter.hpp>

#include <rclcpp/rclcpp.hpp>

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

  using pub_motor_feedback_map_t =
    std::unordered_map<std::string, rclcpp::Publisher<roboclaw_interfaces::msg::MotorFeedback>::SharedPtr>;
  using sub_motor_velocity_map_t =
    std::unordered_map<std::string, rclcpp::Subscription<roboclaw_interfaces::msg::VelocitySetpoint>::SharedPtr>;

  // TODO make service to open driver
  // TODO make service to close driver

  // CLASS MEMBERS --------------------------------------------------------------------------------
  rclcpp::TimerBase::SharedPtr timer_pub_feedback_;
  const ControllerMap controller_map_ {*this};

  std::string device_ = "";
  RoboClaw driver_;
  int baudrate_ = 0;
  double velocity_deadband_ = 0.0;

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

    if (!has_parameter("baudrate"))
    {
      throw std::runtime_error{"No serial 'baudrate' parameter provided, aborting"};
    }

    baudrate_ = get_parameter("baudrate").as_int();

    velocity_deadband_ = std::abs(declare_and_get_parameter<double>(
      *this,
      "velocity.deadband",
      "If velocity set point magnitude is less than this value then the duty cylce is set to 0",
      1.0e-3
    ));

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
        sub_velocity_[controller_name][motor_name] = create_subscription<roboclaw_interfaces::msg::VelocitySetpoint>(
          topic_namespace + "velocity_setpoint",
          rclcpp::QoS{10},
          [this, controller_name, motor_name](const roboclaw_interfaces::msg::VelocitySetpoint& msg)
          {
            sub_velocity_callback(msg, controller_name, motor_name);
          }
        );

        // Create publishers for velocity and position feedback
        pub_feedback_[controller_name][motor_name] = create_publisher<roboclaw_interfaces::msg::MotorFeedback>(
          topic_namespace + "feedback",
          rclcpp::QoS{10}
        );
      }

      RCLCPP_INFO_STREAM(get_logger(), get_fully_qualified_name() << " node started");
    }

    // TIMER --------------------------------------------------------------------------------
    timer_pub_feedback_ = create_wall_timer(
        static_cast<std::chrono::microseconds>(static_cast<int>(100)),
        std::bind(&RoboClawControllerNode::pub_feedback_timer_callback, this)
      );

    // Open driver
    std::cout << "device: " << device_ << "\n";
    std::cout << "baudrate: " << baudrate_ << "\n";
    ReturnCode ret = driver_.open(device_, baudrate_);

    //quit if initialization fails
    if(ret != ReturnCode::OK)
    {
      std::cerr << "unable to initialize roboclaw" <<  static_cast<int>(ret) << std::endl;
      std::exit(1);
      // TODO: make a retry service
    }
  }

  ~RoboClawControllerNode()
  {
    stop_all_motors();
  }

private:

  ReturnCode stop_motor(const roboclaw::ControllerConfig& cfg, roboclaw::MotorSelect motor)
  {
    return driver_.set_duty(cfg, motor, 0.0);
  }

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

  void sub_velocity_callback(
    const roboclaw_interfaces::msg::VelocitySetpoint& velocity,
    const std::string& controller,
    const std::string& motor
  )
  {
    // look for the passed controller in the controller map
    auto target_controller = controller_map_.find(controller);
    if (target_controller != controller_map_.end())
    {
      auto target_controller_info = target_controller->second;
      // look for the passed motor in the controller
      auto target_motor = target_controller_info.motors.find(motor);
      if (target_motor != target_controller_info.motors.end())
      {
        if(std::abs(velocity.set_velocity) < velocity_deadband_){
          stop_motor(*target_controller_info.config, target_motor->second);
        }
        else{
          auto ret = driver_.set_velocity(*target_controller_info.config, target_motor->second, velocity.set_velocity);
          // check that the velocity was set
          if (ret != ReturnCode::OK)
          {
            //TODO change cerr to RCLCPP_ERROR_STREAM("");
            std::cerr << "Could not set velocity point for this controller and motor with error: " << static_cast<int>(ret) << std::endl;
          }
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

  void pub_feedback_timer_callback(){
    for (const auto& controller : controller_map_)
    {
      const auto& controller_name = controller.first;
      const auto& controller_info = controller.second;
      for (const auto& motor : controller_info.motors)
      {
        roboclaw_interfaces::msg::MotorFeedback motor_feedback;

        const auto& motor_name = motor.first;
        auto [ret_vel, velocity] = driver_.get_velocity_radians(*controller_info.config, motor.second);

        // check that velocity was retreived
        if (ret_vel != ReturnCode::OK)
        {
          std::cerr << "unable to get velocity from roboclaw" << std::endl;
        }
        motor_feedback.velocity = velocity;

        auto [ret_pos, position] = driver_.get_enc_radians(*controller_info.config);

        // check that position was retreived
        if (ret_pos != ReturnCode::OK)
        {
          std::cerr << "unable to get position from roboclaw" << std::endl;
        }
        motor_feedback.position = position[to_index(motor.second)]; 

        // publish motor feedback with the current position and velocity
        pub_feedback_[controller_name][motor_name]->publish(motor_feedback);
      }
    }
  }

  // on destruction of node stop all motors (set duty cycle to 0)
  // if we get a velocity set point that is close enough to 0 then stop the motors
  // safe controller, make a time out so if you don't recieve on a topic fo x amount of time for that motor stop the motor
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoboClawControllerNode>());
  rclcpp::shutdown();
  return 0;
}