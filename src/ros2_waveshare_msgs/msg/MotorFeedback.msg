# Complete motor feedback with CIA402 state information and conversion metadata
# Frequency: ~100 Hz (from TPDO callbacks)

std_msgs/Header header

# Motor identification
uint8 node_id                           # CANopen node ID (1-127)
string motor_name                       # Human-readable name (e.g., "traction_motor_left")

# === CIA402 State Machine ===
string state                            # State name: "OPERATION_ENABLED", "FAULT", "NOT_READY", etc.
uint16 statusword                       # Raw statusword value (hex for debugging)
uint8 operation_mode_display            # Current operation mode (1=Position, 3=Velocity, 4=Torque)

# === Measurements (SI Units) ===
float64 position_rad                    # Position in radians
float64 velocity_rad_s                  # Velocity in rad/s
float64 torque_nm                       # Torque in N·m (converted from current)
int16 current_ma                        # Raw current in mA (from TPDO)
float64 voltage_v                       # DC link voltage in Volts (if available)
float64 temperature_c                   # Motor/driver temperature in °C (if available)

# === Status Flags (decoded from statusword) ===
bool ready_to_switch_on
bool switched_on
bool operation_enabled
bool fault
bool voltage_enabled
bool warning
bool target_reached
bool remote                             # Remote control enabled
bool quick_stop_active

# === Error Information ===
uint8 error_register                    # CIA402 error register (0x1001)

# === Conversion Metadata (for diagnostics/visualization) ===
# These constants show how raw counts were converted to SI units
float64 counts_per_revolution           # Encoder resolution
float64 gear_ratio                      # Gearbox reduction ratio
float64 torque_constant_nm_per_a        # Torque constant (N·m per Ampere)
uint32 encoder_counts                   # Raw encoder counts (before conversion)
int32 velocity_counts_per_sec           # Raw velocity (before conversion)

# === Timestamps ===
builtin_interfaces/Time last_tpdo1_time # Last TPDO1 received timestamp
builtin_interfaces/Time last_tpdo2_time # Last TPDO2 received timestamp
