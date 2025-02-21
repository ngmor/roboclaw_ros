#
# Feedback from a RoboClaw motor channel
#

# Current velocity (rad/s) of the motor
float64 velocity

# If true, velocity read was valid
bool valid_velocity

# Current position (rad) of the motor
float64 position

# If true, position read was valid
bool valid_positon

# The time that this message was defined by the RoboClaw
# The seconds component, valid over all int32 values.
int32 sec
# The nanoseconds component, valid in the range [0, 10e9).
uint32 nanosec