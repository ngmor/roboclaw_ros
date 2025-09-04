# RoboClaw ROS 2 Driver
This is a wrapper around this [RoboClaw driver](https://github.com/ngmor/roboclaw_driver) for use in ROS 2.

## Workspace Setup
Setup a typical ROS 2 workspace:

```bash
# Replace ${ws} with a path to your workspace
mkdir -p ${ws}/src
cd ${ws}/src

# Clone required repositories (can use HTTPS too of course)
git clone git@github.com:ngmor/roboclaw_driver.git
git clone git@github.com:ngmor/roboclaw_ros.git
```

## Build
Build as typical with ROS 2:
```bash
cd ${ws}
colcon build
```

## Usage
### Basic Configuration
To run the roboclaw node, a configuration YAML file _must_ be provided which configures the controllers and channels that are available on the serial port. An example with explanation can be found [here](roboclaw_ros/config/controllers.yaml).

### Running the node
```bash
cd ${ws}

# Source workspace (after build)
. install/setup.bash

# Run the node
# Replace ${controller_config_path} with the path to the controller
# configuration file you'd like to use
ros2 run roboclaw_ros roboclaw_controller --ros-args --params-file ${controller_config_path}
```

### Node Interfaces
Currently the node only has basic features, more features are planned in the future (but PRs are always welcome!).

#### Subscribers
- `/<controller_name>/<channel_name>/velocity_setpoint` [roboclaw_interfaces/msg/VelocitySetpoint]: a topic following this pattern is created for every controller/channel configured in the controller config YAML. Messages published to this topic will control that motor's velocity (rad/s). If the node does not receive setpoints on this topic within a certain timeout, the node will command a stop of the motor.

#### Publishers
- `/<controller_name>/<channel_name>/feedback` [roboclaw_interfaces/msg/MotorFeedback]: feedback published at a configurable rate about the motor channel.

#### Services
- `/open_port` [std_srvs/srv/Trigger]: manually open the serial port (attempted on node startup)
- `/close_port` [std_srvs/srv/Trigger]: manually close the serial port

### Further Configuration
More parameters are available to configure the node's behavior. To see descriptions, run the following while the node is active:
```bash
# replace ${node_name} with name of node
# (default: /roboclaw_controller)
ros2 param describe ${node_name} $(ros2 param list ${node_name})
```