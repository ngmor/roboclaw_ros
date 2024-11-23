#include <roboclaw_ros/utils/controller_map.hpp>

namespace roboclaw
{

namespace utils
{

enc_count_t ControllerInfo::enc_counts_per_rev(MotorSelect motor) const
{
  return config->enc_counts_per_rev(motor);
}

enc_count_t ControllerInfo::enc_counts_per_rev(const std::string& motor) const
{
  return config->enc_counts_per_rev(motors.at(motor));
}

ControllerConfig::address_t ControllerInfo::address() const
{
  return config->address();
}

std::ostream& operator<<(std::ostream& os, const ControllerInfo& controller)
{
  os << controller.name << ":\t{ Address: 0x" << std::hex << static_cast<int>(controller.address());
  for (const auto& motor : controller.motors)
  {
    const auto channel = motor.second == MotorSelect::M1 ? "M1" : "M2";

    os << ", " << motor.first << " (" << channel << ")";
  }
  os << "}";
  return os;
}

ControllerMap::ControllerMap(const rclcpp::Node& node)
{
  const std::string controllers_param_name = "controllers";

  // Get all parameters starting with "controllers" and iterate through them
  std::map<std::string, rclcpp::Parameter> controller_params;
  node.get_parameters(controllers_param_name, controller_params);

  for (const auto& param : controller_params)
  {
    // Parse the controller name from the parameter name (should be
    // the first item, before the first ".")
    const std::string controller_name = param.first.substr(0, param.first.find("."));

    // If we've already processed a controller with this name, skip
    if (controllers_.find(controller_name) != controllers_.end())
    {
      continue;
    }

    const std::string controller_param = controllers_param_name + "." + controller_name;

    // Function to get a member parameter of the current controller
    auto get_member = [&](const std::string& name)
    {
      const std::string full_member_name = controller_param + "." + name;

        if (!node.has_parameter(full_member_name))
        {
          throw rclcpp::exceptions::ParameterNotDeclaredException(
            "Malformed controller config: Controller " + controller_name
            + " does not have " + name
          );
        }

        return node.get_parameter(full_member_name);
    };

    // Populate controller struct with data
    ControllerInfo controller;
    controller.name = controller_name;

    const auto address = static_cast<ControllerConfig::address_t>(get_member("address").as_int());

    enc_count_t M1_enc_counts_per_rev = 0;
    enc_count_t M2_enc_counts_per_rev = 0;

    // Only add motor channels if they exist in config
    if (node.has_parameter(controller_param + ".M1.name"))
    {
      const auto motor_name = get_member("M1.name").as_string();
      controller.motors[motor_name] = MotorSelect::M1;
      M1_enc_counts_per_rev =
        static_cast<enc_count_t>(get_member("M1.enc_counts_per_rev").as_int());
    }
    if (node.has_parameter(controller_param + ".M2.name"))
    {
      const auto motor_name = get_member("M2.name").as_string();
      controller.motors[motor_name] = MotorSelect::M2;
      M2_enc_counts_per_rev =
        static_cast<enc_count_t>(get_member("M2.enc_counts_per_rev").as_int());
    }

    controller.config = std::make_shared<ControllerConfig>(
      address,
      M1_enc_counts_per_rev,
      M2_enc_counts_per_rev
    );

    // Add controller to internal map
    controllers_[controller_name] = controller;
  }
}

std::ostream& operator<<(std::ostream& os, const ControllerMap& controllers)
{
  os << "Controllers (" << controllers.size() << "):" << std::endl;
  for (const auto& controller : controllers)
  {
    os << "\t" << controller.second << std::endl;
  }
  return os;
}

}  // namespace utils

}  // namespace roboclaw