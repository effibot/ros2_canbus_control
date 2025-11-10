# ROS2 Interfaces Design for waveshare_cpp CANopen Wrapper

> **Author**: effibot (andrea.efficace1@gmail.com)  
> **Date**: 2025-11-10  
> **Purpose**: Complete design specification for ROS2 messages, services, and actions

## Table of Contents

1. [Design Philosophy](#design-philosophy)
2. [Standard vs Custom Messages](#standard-vs-custom-messages)
3. [Message Definitions](#message-definitions)
4. [Service Definitions](#service-definitions)
5. [Action Definitions](#action-definitions)
6. [Topic Architecture](#topic-architecture)
7. [Implementation Plan](#implementation-plan)

---

## Design Philosophy

### Core Principles

1. **Prefer Standard Messages** - Use `sensor_msgs`, `control_msgs`, `std_msgs` when possible
2. **Type Safety** - Mirror the enum-first design from waveshare_cpp
3. **Performance** - Minimize message complexity for real-time PDO data
4. **Diagnostics** - Rich error reporting using `diagnostic_msgs`
5. **Multi-Motor Ready** - Design for multiple motors on same bus

### Message Complexity Trade-offs

| Message Type     | Complexity | Frequency | Standard?                      |
| ---------------- | ---------- | --------- | ------------------------------ |
| Joint feedback   | Medium     | 100+ Hz   | ✅ `sensor_msgs/JointState`     |
| Motor commands   | Low        | 10-100 Hz | ✅ `std_msgs/Float64` or custom |
| State monitoring | Low        | 1-10 Hz   | ❌ Custom                       |
| SDO read/write   | Medium     | On-demand | ❌ Custom service               |
| Motor enable     | Medium     | Rare      | ❌ Custom action                |

---

## Standard vs Custom Messages

### Use Standard Messages For:

#### 1. Joint Feedback → `sensor_msgs/JointState`

**Why**: Industry standard for robot joint data, widely supported.

```yaml
# sensor_msgs/msg/JointState
std_msgs/Header header
string[] name          # ["motor_1", "motor_2"]
float64[] position     # rad or meters
float64[] velocity     # rad/s or m/s  
float64[] effort       # N·m or N (torque/force)
```

**Mapping from CANopen**:
- `position` ← TPDO1 Position Actual Value (convert counts → rad)
- `velocity` ← TPDO2 Velocity Actual Value (convert counts/s → rad/s)
- `effort` ← TPDO2 Current Actual Value (convert mA → N·m using torque constant)

**Advantages**:
- ✅ Works with `robot_state_publisher`, `joint_state_publisher`
- ✅ Compatible with `MoveIt`, `ros2_control`
- ✅ Standard plotting tools (PlotJuggler, rqt_plot)

#### 2. Diagnostics → `diagnostic_msgs/DiagnosticArray`

**Why**: Standard ROS2 health monitoring.

```yaml
# diagnostic_msgs/msg/DiagnosticStatus
byte OK = 0
byte WARN = 1
byte ERROR = 2
byte STALE = 3

byte level
string name
string message
string hardware_id
diagnostic_msgs/KeyValue[] values  # key-value pairs
```

**Usage for Motor Diagnostics**:
```
Level: ERROR
Name: "motor_1_status"
Message: "Motor fault detected"
Hardware ID: "traction_motor_left"
Values:
  - key: "statusword", value: "0x0008"
  - key: "error_register", value: "0x01"
  - key: "state", value: "FAULT"
```

---

### Create Custom Messages For:

#### 1. Motor Feedback (Detailed)

**Need**: More information than `JointState` provides (statusword, state, error flags).

**Proposal**: `ros2_waveshare_msgs/MotorFeedback`

```
# MotorFeedback.msg
std_msgs/Header header

uint8 node_id                    # CANopen node ID (1-127)
string motor_name                # Human-readable name

# CIA402 State
string state                     # "OPERATION_ENABLED", "FAULT", etc.
uint16 statusword                # Raw statusword value

# Position/Velocity/Torque (converted to SI units)
float64 position_rad             # Position in radians
float64 velocity_rad_s           # Velocity in rad/s
float64 torque_nm                # Torque in N·m
int16 current_ma                 # Raw current in mA

# Flags (decoded from statusword)
bool ready_to_switch_on
bool switched_on
bool operation_enabled
bool fault
bool voltage_enabled
bool warning
bool target_reached

# Error information
uint8 error_register             # CIA402 error register
```

**Usage**: Detailed monitoring, debugging, state machine visualization.

#### 2. Motor Command

**Need**: Send position/velocity/torque commands with controlword.

**Proposal**: `ros2_waveshare_msgs/MotorCommand`

```
# MotorCommand.msg
std_msgs/Header header

uint8 node_id                    # CANopen node ID (1-127)

# Operation mode (matches CIA402 enum)
int8 operation_mode              # 1=Position, 3=Velocity, 4=Torque
# int8 NO_MODE = 0
# int8 PROFILE_POSITION = 1
# int8 PROFILE_VELOCITY = 3
# int8 PROFILE_TORQUE = 4

# Target values (use based on operation_mode)
float64 target_position_rad      # For Profile Position mode
float64 target_velocity_rad_s    # For Profile Velocity mode
float64 target_torque_nm         # For Profile Torque mode

# Control flags (optional, usually set automatically)
uint16 controlword               # Raw controlword (0 = auto)
```

**Usage**: High-level command interface for applications.

---

## Message Definitions

### Package Structure

```
ros2_waveshare_msgs/
├── CMakeLists.txt
├── package.xml
├── msg/
│   ├── MotorFeedback.msg
│   ├── MotorCommand.msg
│   ├── MotorStatus.msg           # Lightweight status
│   └── PDOStatistics.msg         # PDO performance metrics
├── srv/
│   ├── SDORead.srv
│   ├── SDOWrite.srv
│   ├── SetOperationMode.srv
│   └── GetMotorInfo.srv
└── action/
    ├── EnableMotor.action
    ├── ResetFault.action
    └── MoveToPosition.action      # High-level motion
```

### msg/MotorFeedback.msg

```
# Complete motor feedback with CIA402 state information
# Published at PDO rate (100+ Hz)
std_msgs/Header header

uint8 node_id
string motor_name

# CIA402 State Machine
string state                     # State name (human-readable)
uint16 statusword                # Raw statusword value (hex)
uint8 operation_mode_display     # Current operation mode

# Measurements (SI units)
float64 position_rad             # Position in radians
float64 velocity_rad_s           # Velocity in rad/s
float64 acceleration_rad_s2      # Acceleration in rad/s² (computed)
float64 torque_nm                # Torque in N·m (computed from current)
int16 current_ma                 # Raw current in mA

# Additional measurements (from CIA402 registers)
float64 position_demand_rad      # Trajectory generator position (0x6062)
float64 velocity_demand_rad_s    # Trajectory generator velocity (0x6043)
float64 position_error_rad       # Position error (target - actual)

# Motion profile parameters (for SLAM/odometry)
float64 profile_velocity_rad_s   # Max velocity for position mode (0x6081)
float64 profile_acceleration_rad_s2  # Acceleration limit (0x6083)

# Temperature and voltage (if available)
int16 motor_temperature_c        # Motor temperature in °C (if supported)
uint16 dc_link_voltage_mv        # DC bus voltage in mV (if supported)

# Status flags (decoded from statusword)
bool ready_to_switch_on
bool switched_on
bool operation_enabled
bool fault
bool voltage_enabled
bool warning
bool target_reached
bool remote
bool quick_stop_active
bool internal_limit_active

# Error information
uint8 error_register

# Conversion constants (for diagnostics/debugging)
float64 counts_per_revolution    # Encoder resolution
float64 gear_ratio               # Gearbox reduction
float64 torque_constant_nm_a     # Torque constant (N·m/A)
```

### msg/MotorCommand.msg

```
# Motor command message (position, velocity, or torque)
std_msgs/Header header

uint8 node_id

# Operation mode selection
int8 operation_mode
# Constants (match CIA402 OperationMode enum)
int8 NO_MODE = 0
int8 PROFILE_POSITION = 1
int8 PROFILE_VELOCITY = 3
int8 PROFILE_TORQUE = 4
int8 HOMING = 6
int8 CYCLIC_SYNC_POSITION = 8
int8 CYCLIC_SYNC_VELOCITY = 9

# Command values (use based on operation_mode)
float64 target_position_rad
float64 target_velocity_rad_s
float64 target_torque_nm

# Optional controlword override (0 = automatic)
uint16 controlword
```

### msg/MotorStatus.msg

```
# Lightweight motor status (for high-frequency publishing)
std_msgs/Header header

uint8 node_id
string state                     # "OPERATION_ENABLED", "FAULT", etc.
bool fault
bool warning
bool target_reached
```

### msg/PDOStatistics.msg

```
# PDO Manager performance statistics
std_msgs/Header header

uint64 tpdo1_received
uint64 tpdo2_received
uint64 rpdo1_sent
uint64 rpdo2_sent
uint64 errors
float64 avg_latency_us
```

---

## Service Definitions

### srv/SDORead.srv

```
# Request
uint8 node_id
string object_name               # From object dictionary (e.g., "statusword")
# OR
uint16 index                     # Direct index access (if object_name empty)
uint8 subindex
---
# Response
bool success
string message                   # Error message if failed
bytes data                       # Raw bytes read
# Parsed values (if success)
uint8 value_u8
uint16 value_u16
uint32 value_u32
int8 value_i8
int16 value_i16
int32 value_i32
```

**Usage**:
```bash
ros2 service call /motor_1/sdo/read ros2_waveshare_msgs/srv/SDORead "{node_id: 1, object_name: 'statusword'}"
```

### srv/SDOWrite.srv

```
# Request
uint8 node_id
string object_name               # From object dictionary
# OR
uint16 index                     # Direct index access
uint8 subindex

# Value to write (use appropriate field)
uint8 value_u8
uint16 value_u16
uint32 value_u32
int8 value_i8
int16 value_i16
int32 value_i32
---
# Response
bool success
string message
```

**Usage**:
```bash
ros2 service call /motor_1/sdo/write ros2_waveshare_msgs/srv/SDOWrite \
  "{node_id: 1, object_name: 'target_position', value_i32: 10000}"
```

### srv/SetOperationMode.srv

```
# Request
uint8 node_id
int8 operation_mode              # 1=Position, 3=Velocity, 4=Torque
---
# Response
bool success
string message
int8 current_mode                # Actual mode after setting
```

### srv/GetMotorInfo.srv

```
# Request
uint8 node_id
---
# Response
bool success
string motor_name
uint32 device_type
uint32 vendor_id
uint32 product_code
string firmware_version
string state                     # Current CIA402 state
```

---

## Action Definitions

### action/EnableMotor.action

```
# Goal
uint8 node_id
---
# Result
bool success
string message
string final_state               # Should be "OPERATION_ENABLED"
---
# Feedback
string current_state
uint16 statusword
float32 progress_percent         # 0-100%
```

**State Transition Sequence**:
```
NOT_READY (0%)
  ↓
SWITCH_ON_DISABLED (25%)
  ↓
READY_TO_SWITCH_ON (50%)
  ↓
SWITCHED_ON (75%)
  ↓
OPERATION_ENABLED (100%) ✅
```

**Usage**:
```bash
ros2 action send_goal /motor_1/enable ros2_waveshare_msgs/action/EnableMotor "{node_id: 1}"
```

### action/ResetFault.action

```
# Goal
uint8 node_id
---
# Result
bool success
string message
uint8 error_register_before
uint8 error_register_after
---
# Feedback
string current_state
uint8 error_register
```

### action/MoveToPosition.action

```
# Goal
uint8 node_id
float64 target_position_rad
float64 max_velocity_rad_s       # Optional velocity limit
float64 timeout_sec              # Max time to reach target
---
# Result
bool success
string message
float64 final_position_rad
float64 position_error_rad
---
# Feedback
float64 current_position_rad
float64 current_velocity_rad_s
float64 distance_to_target_rad
float32 progress_percent
bool target_reached
```

**Usage**:
```bash
ros2 action send_goal /motor_1/move_to_position ros2_waveshare_msgs/action/MoveToPosition \
  "{node_id: 1, target_position_rad: 1.57, max_velocity_rad_s: 0.5, timeout_sec: 5.0}"
```

---

## Topic Architecture

### MotorDriverNode Topic Structure

```
/motors/                         # Namespace for all motors
  ├── motor_1/
  │   ├── feedback               # MotorFeedback (100Hz)
  │   ├── command                # MotorCommand (subscribe)
  │   ├── status                 # MotorStatus (10Hz, lightweight)
  │   ├── sdo/
  │   │   ├── read               # Service
  │   │   └── write              # Service
  │   ├── enable                 # Action
  │   ├── reset_fault            # Action
  │   └── move_to_position       # Action
  │
  ├── motor_2/
  │   ├── feedback
  │   ├── command
  │   └── ...
  │
  ├── motor_3/
  │   └── ...
  │
  ├── motor_4/
  │   └── ...
  │
  ├── joint_states               # sensor_msgs/JointState (all motors combined, 100Hz)
  └── diagnostics                # diagnostic_msgs/DiagnosticArray (1Hz)

/bridge/
  └── diagnostics                # Bridge statistics
```

### Multi-Motor System Example (Your Traction Robot)

```
/motors/
  ├── traction_left/             # Left wheel motor
  │   ├── feedback
  │   ├── command
  │   └── ...
  │
  ├── traction_right/            # Right wheel motor
  │   ├── feedback
  │   ├── command
  │   └── ...
  │
  ├── steering_front/            # Front steering motor (if applicable)
  │   ├── feedback
  │   └── ...
  │
  ├── steering_rear/             # Rear steering motor (if applicable)
  │   └── ...
  │
  └── joint_states               # Combined: [traction_left, traction_right, steering_front, steering_rear]

/cmd_vel                         # geometry_msgs/Twist (from your dynamics node)
/odom                            # nav_msgs/Odometry (from your dynamics node → SLAM)
```

**Benefits of `/motors/` namespace**:
- ✅ Clean organization (all motors under one namespace)
- ✅ Scalable (supports 4+ motors easily)
- ✅ Clear hierarchy (motor name indicates purpose)
- ✅ Avoids topic pollution at root level

---

## Implementation Plan

### Phase 1: Create Interface Package ✅ NEXT

1. **Create `ros2_waveshare_msgs` package**
   ```bash
   ros2 pkg create ros2_waveshare_msgs \
     --build-type ament_cmake \
     --dependencies rosidl_default_generators
   ```

2. **Add message definitions**
   - `msg/MotorFeedback.msg`
   - `msg/MotorCommand.msg`
   - `msg/MotorStatus.msg`
   - `msg/PDOStatistics.msg`

3. **Add service definitions**
   - `srv/SDORead.srv`
   - `srv/SDOWrite.srv`
   - `srv/SetOperationMode.srv`
   - `srv/GetMotorInfo.srv`

4. **Add action definitions**
   - `action/EnableMotor.action`
   - `action/ResetFault.action`
   - `action/MoveToPosition.action`

5. **Update `CMakeLists.txt` and `package.xml`**

6. **Build and verify**
   ```bash
   colcon build --packages-select ros2_waveshare_msgs
   source install/setup.bash
   ros2 interface list | grep ros2_waveshare_msgs
   ```

### Phase 2: Implement MotorDriverNode

1. **Design node architecture**
   - Component diagram
   - Thread model
   - Parameter structure

2. **Implement core functionality**
   - SDO/PDO/FSM integration
   - Topic publishers/subscribers
   - Service servers
   - Action servers

3. **Add parameter handling**
   - Motor configuration
   - Unit conversions
   - Performance tuning

4. **Testing**
   - Unit tests (mock CAN socket)
   - Integration tests (vcan)
   - Hardware tests (real motor)

### Phase 3: Launch Files & Configuration

1. **Motor parameter files**
   - `traction_motor_left.yaml`
   - `traction_motor_right.yaml`

2. **Launch files**
   - `motor_driver.launch.py` (single motor)
   - `dual_motor.launch.py` (traction system)
   - `full_system.launch.py` (bridge + motors)

3. **Documentation**
   - User guide
   - Configuration guide
   - Troubleshooting

---

## Design Decisions

### Why Custom Messages?

**Rationale**: While `sensor_msgs/JointState` is great for feedback, motor control requires:
- CIA402 state machine visibility
- Raw statusword/controlword access
- Error register monitoring
- Operation mode control

**Benefit**: Both standard (`JointState`) and custom (`MotorFeedback`) published simultaneously:
- Standard messages for generic tools
- Custom messages for detailed monitoring

### Why Actions for Enable/Reset?

**Rationale**: Motor enable and fault reset are multi-step sequences:
- Enable: 4-5 state transitions (5+ seconds)
- Reset: Fault detection + recovery (2+ seconds)

**Benefit**: 
- ✅ Cancellable (user can abort)
- ✅ Progress feedback (state transitions visible)
- ✅ Timeout handling (built-in)
- ✅ Standard pattern (familiar to ROS2 users)

### Why Services for SDO?

**Rationale**: SDO reads/writes are request-response operations:
- Low frequency (on-demand configuration)
- Synchronous (caller waits for result)
- Error-prone (need success/failure status)

**Benefit**:
- ✅ Simple API (request → response)
- ✅ Clear error handling
- ✅ No message queuing issues

---

## Unit Conversion Strategy

### Encoder Counts ↔ Radians

```
position_rad = (counts / counts_per_revolution) * 2π
velocity_rad_s = (counts_per_sec / counts_per_revolution) * 2π
```

**Parameters** (from `motor_params.yaml`):
```yaml
motor_1:
  counts_per_revolution: 10000      # Encoder resolution
  gear_ratio: 50.0                  # Reduction ratio
  torque_constant: 0.05             # N·m per Ampere
```

### Current → Torque

```
torque_nm = (current_ma / 1000.0) * torque_constant
```

---

## Next Steps

1. ✅ **Review this design** - Feedback on message/service/action structure
2. **Create `ros2_waveshare_msgs` package** - Implement all interfaces
3. **Implement `MotorDriverNode`** - Full CANopen wrapper
4. **Test with real hardware** - Validate design choices

---

**Last Updated**: 2025-11-10  
**Status**: Design complete, awaiting implementation
