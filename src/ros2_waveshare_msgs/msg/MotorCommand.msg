# Motor command message (position, velocity, or torque)
# Subscribe to this topic to send commands to motor

std_msgs/Header header

uint8 node_id                           # CANopen node ID (1-127)

# === Operation Mode Selection ===
int8 operation_mode
# Constants (match CIA402 OperationMode enum)
int8 NO_MODE = 0
int8 PROFILE_POSITION = 1
int8 PROFILE_VELOCITY = 3
int8 PROFILE_TORQUE = 4
int8 HOMING = 6
int8 INTERPOLATED_POSITION = 7
int8 CYCLIC_SYNC_POSITION = 8
int8 CYCLIC_SYNC_VELOCITY = 9
int8 CYCLIC_SYNC_TORQUE = 10

# === Command Values (use based on operation_mode) ===
float64 target_position_rad             # For Profile Position mode (radians)
float64 target_velocity_rad_s           # For Profile Velocity mode (rad/s)
float64 target_torque_nm                # For Profile Torque mode (N·m)

# === Advanced Control (optional) ===
uint16 controlword                      # Raw controlword override (0 = automatic)
float64 max_velocity_rad_s              # Velocity limit for position mode (0 = use default)
float64 max_acceleration_rad_s2         # Acceleration limit (0 = use default)
