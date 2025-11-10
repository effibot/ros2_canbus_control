# Lightweight motor status for high-frequency publishing
# Frequency: ~10 Hz (less detailed than MotorFeedback)

std_msgs/Header header

uint8 node_id
string motor_name
string state                            # "OPERATION_ENABLED", "FAULT", etc.
bool fault
bool warning
bool target_reached
bool operational                        # True if in OPERATION_ENABLED state
