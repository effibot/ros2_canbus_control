# ROS2 CAN Bus Control System - AI Coding Instructions

## Project Overview

ROS2 middleware for industrial motor control using Waveshare USB-CAN-A adapters. Implements complete CANopen stack with CIA402 motor control for MICROPHASE TRAC_PWR servo drives.

**Status**: ✅ CANopen layer complete, ✅ Motor driver node implemented with PDO/SDO, 🚧 Action handlers in progress (enable motor, fault reset, position moves).

**Critical Architecture**: `USB Hardware → Serial Protocol → SocketCAN Bridge → CANopen Stack → ROS2 Motor Driver Node`

## Core Components

### 1. Waveshare C++ Library (`src/ros2_waveshare/lib/waveshare_cpp/`) - Git Submodule
- **State-First Architecture** with CRTP patterns (see library's own `.github/copilot-instructions.md`)
- Serial I/O via POSIX termios2 (up to 2Mbps)
- Thread-safe: 3-mutex pattern in USBAdapter, lock-free atomics in SocketCANBridge/PDOManager
- **CANopen complete**: SDOClient, PDOManager, CIA402FSM, ObjectDictionary (enum-first design, NO magic numbers)

### 2. ROS2 Lifecycle Bridge (`src/ros2_waveshare/canopen_lifecycle*`) - Package: `ros2_waveshare`
- `CanopenLifeCycleNode`: Manages SocketCAN bridge lifecycle  
- Wraps `SocketCANBridge` from waveshare_cpp library
- Launch: `bridge_bringup.launch.py` with `bridge_params.yaml`

### 3. Motor Driver Node (`src/ros2_waveshare/motor_driver_*`) - Package: `ros2_waveshare`
- **Architecture**: Single node manages multiple motors (1-4) on shared CAN bus
- **File Structure** (modular design):
  - `motor_driver_node.cpp` - Initialization (CAN, motors, PDO, publishers, timers)
  - `motor_driver_handlers.cpp` - ROS2 service callbacks (SDO read/write, mode setting, motor info)
  - `motor_driver_callbacks.cpp` - PDO callbacks, timer callbacks, message builders
  - `motor_instance.{cpp,hpp}` - Per-motor state container with thread-safe accessors
- **ROS2 Interfaces** (per motor):
  - Services: `/motors/motor_N/{sdo/read, sdo/write, set_operation_mode, get_motor_info}`
  - Actions: `/motors/motor_N/{enable, reset_fault, move_to_position}`
  - Publishers: `/motors/motor_N/{feedback, status}`, `/joint_states`, `/diagnostics`
  - Subscribers: `/motors/motor_N/command`

### 4. CANopen Configuration (`src/microphase_can_config/`) - Package: `microphase_can_config`
- Bus config: `bus.yml` defines 4 nodes (IDs 2-5) + master (ID 1)
- EDS file: `config/can_config/eds/TRACTION_PWR.eds` (CIA402 compliant)
- Unified launch: `microphase_can_config.launch.py` starts bridge + CANopen stack

## Critical Development Patterns

### Library Helper Usage (MANDATORY Pattern)

**Core Principle**: NEVER manually manipulate bytes for CANopen data. Always use library helpers.

```cpp
// ✅ CORRECT: Type-safe byte conversion
auto& dict = motor->get_dictionary();

// Writing SDO
uint16_t controlword = 0x000F;
std::vector<uint8_t> data = dict.to_raw(controlword);  // Handles endianness
sdo_client->write_object("controlword", data);

// Reading SDO
auto data = sdo_client->read_object("statusword");
uint16_t statusword = dict.from_raw<uint16_t>(data);  // Type-safe parsing

// ❌ WRONG: Manual byte manipulation
data.push_back(controlword & 0xFF);        // DON'T DO THIS
data.push_back((controlword >> 8) & 0xFF); // Library handles it!
```

**CIA402 State Helpers** (always use for user-facing messages):
```cpp
// Decode statusword to enum
auto state = canopen::cia402::decode_statusword(statusword);

// Get human-readable strings
const char* state_name = canopen::cia402::get_state_description(state);
// Returns: "Operation Enabled", "Fault", "Switch On Disabled", etc.

const char* mode_desc = canopen::cia402::get_mode_description(mode_value);
// Returns: "Profile Position (PP)", "Cyclic Sync Velocity (CSV)", etc.

// ✅ Enhanced error messages
RCLCPP_ERROR(logger, "Mode verification failed: requested %d (%s) but got %d (%s)",
    requested, canopen::cia402::get_mode_description(requested),
    actual, canopen::cia402::get_mode_description(actual));
```

**Supported Types**: `uint8_t`, `int8_t`, `uint16_t`, `int16_t`, `uint32_t`, `int32_t`  
**Template Instantiations**: Explicitly defined in `object_dictionary.cpp` - compiler will error if you try unsupported types.

### Motor Driver Node Architecture

**Initialization Sequence** (constructor):
1. `load_parameters()` - Read ROS2 params (CAN interface, rates, limits, etc.)
2. `initialize_can_socket()` - Open shared `RealCANSocket` for all motors
3. `initialize_motors()` - For each motor:
   - Load object dictionary from JSON config
   - Create `SDOClient`, `CIA402FSM` instances
   - Setup per-motor ROS2 interfaces (publishers, subscribers, services, actions)
4. `initialize_pdo_manager()` - Single `PDOManager` for all motors
   - Register TPDO1/TPDO2 callbacks per motor
   - Start receive thread
5. `setup_publishers()` - Combined publishers (`/joint_states`, `/diagnostics`, `/pdo_statistics`)
6. `setup_timers()` - SYNC (100Hz), JointState (100Hz), Diagnostics (1Hz)

**PDO Callback Pattern** (real-time feedback processing):
```cpp
void MotorDriverNode::on_tpdo1_received(uint8_t node_id, const can_frame& frame) {
    auto motor = get_motor(node_id);
    auto& dict = motor->get_dictionary();
    
    // Parse TPDO1: Statusword (bytes 0-1) + Position (bytes 2-5)
    std::vector<uint8_t> statusword_data(frame.data, frame.data + 2);
    uint16_t statusword = dict.from_raw<uint16_t>(statusword_data);
    
    std::vector<uint8_t> position_data(frame.data + 2, frame.data + 6);
    int32_t position_counts = dict.from_raw<int32_t>(position_data);
    
    // Update motor state (thread-safe)
    motor->update_statusword(statusword);
    motor->update_position(position_counts);
    motor->update_last_tpdo1_time(this->now());
    
    // Publish individual motor feedback
    auto msg = build_motor_feedback(node_id);
    motor->get_feedback_publisher()->publish(msg);
}
```

**Service Handler Pattern** (synchronous SDO operations):
```cpp
void MotorDriverNode::handle_sdo_write(...) {
    auto motor = get_motor(node_id);
    auto& dict = motor->get_dictionary();
    
    // Convert request value to bytes using library helper
    std::vector<uint8_t> data;
    switch (request->value_type) {
        case "uint16": data = dict.to_raw(request->value_u16); break;
        case "int32":  data = dict.to_raw(request->value_i32); break;
        // ... etc
    }
    
    // SDO write (blocks until complete or timeout)
    motor->get_sdo_client()->write_object(request->object_name, data);
    
    response->success = true;
}
```

### ROS2 Workspace Management

**Build Commands** (always from workspace root `/home/ros/ws`):
```bash
# Custom alias (defined in shell)
ros_build_pkg ros2_waveshare              # Incremental build, single package
ros_build_pkg ros2_waveshare microphase_can_config  # Multiple packages

# Standard colcon (if alias unavailable)
colcon build --packages-select ros2_waveshare --symlink-install --merge-install

# Clean rebuild
rm -rf build/ros2_waveshare install/ros2_waveshare
colcon build --packages-select ros2_waveshare --symlink-install --merge-install

# Source after every build
source install/setup.bash
```

**File Organization**:
```
src/ros2_waveshare/
├── src/
│   ├── canopen_lifecycle.cpp        # Lifecycle bridge node
│   ├── motor_driver_node.cpp        # Motor driver initialization
│   ├── motor_driver_handlers.cpp    # Service callbacks (SDO, mode, info)
│   ├── motor_driver_callbacks.cpp   # PDO callbacks, timers, message builders
│   └── motor_instance.cpp            # Per-motor state container
├── include/ros2_waveshare/
│   ├── canopen_lifecycle.hpp
│   ├── motor_driver_node.hpp         # Main node header (400+ lines)
│   └── motor_instance.hpp            # Motor state + ROS2 interface holders
├── lib/waveshare_cpp/                # Git submodule (see its own copilot-instructions.md)
└── CMakeLists.txt                    # Auto-collects src/motor_*.cpp
```

**CMake Pattern**: Uses `file(GLOB_RECURSE MOTOR_DRIVER_SOURCES "src/motor_*.cpp")` - automatically picks up new `motor_*.cpp` files. No manual CMake edits needed for new motor driver source files.

### SocketCAN Bridge Setup

**Required One-Time Setup**:
```bash
# 1. Load kernel modules
sudo modprobe can can-raw can-bcm vcan

# 2. Persist at boot
echo "can\ncan-raw\ncan-bcm\nvcan" | sudo tee /etc/modules-load.d/can.conf

# 3. Create virtual CAN (testing without hardware)
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# 4. USB permissions
echo 'KERNEL=="ttyUSB[0-9]*",MODE="0666"' | sudo tee /etc/udev/rules.d/50-myusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**Bridge Operation**: Node creates `/dev/ttyUSB0` ↔ `vcan0` bidirectional forwarding
- **Configure**: Validates USB device, reads YAML params
- **Activate**: Opens serial + SocketCAN, starts forwarding threads
- **Deactivate**: Stops threads, closes connections

### Lifecycle Node Launch Pattern

**Auto-Activation in Launch Files**:
```python
# bridge_bringup.launch.py snippet
lifecycle_node = LifecycleNode(
    package='ros2_waveshare',
    executable='canopen_lifecycle_node',
    name='waveshare_bridge',
    parameters=[bridge_params],
    output='screen',
)

# Automatic transition to active state
configure_event = EmitEvent(event=ChangeState(
    lifecycle_node_matcher=matches_action(lifecycle_node),
    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
))

activate_event = RegisterEventHandler(
    OnStateTransition(
        target_lifecycle_node=lifecycle_node,
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(lifecycle_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        ))],
    )
)
```

**Critical**: Always use `matches_action(node_instance)` instead of `matches_node_name()` to avoid duplicate node detection issues in multi-launch environments.

## Debugging Workflows

### Hardware-Independent Testing
```bash
# Waveshare library tests (132 tests, no hardware)
cd src/ros2_waveshare/lib/waveshare_cpp
cmake -B build && cmake --build build
ctest --test-dir build --output-on-failure

# SocketCAN simulation (vcan0)
candump vcan0                    # Terminal 1: Monitor
cansend vcan0 "123#DEADBEEF"     # Terminal 2: Send test frames
```

### Motor Driver Node Testing
```bash
# Launch full system
ros2 launch microphase_can_config microphase_can_config.launch.py

# Verify nodes active
ros2 lifecycle get /waveshare_bridge  # Should be: active
ros2 node list | grep motor_driver    # Should show motor_driver node

# Test SDO services (works without motors in simulation)
ros2 service call /motors/motor_1/sdo/write ros2_waveshare_msgs/srv/SDOWrite \
  "{object_name: 'controlword', value_u16: 6, value_type: 'uint16'}"

ros2 service call /motors/motor_1/sdo/read ros2_waveshare_msgs/srv/SDORead \
  "{object_name: 'statusword'}"

# Monitor feedback (100Hz when motors connected)
ros2 topic echo /motors/motor_1/feedback --no-arr
ros2 topic echo /joint_states

# Check PDO statistics
ros2 topic echo /motors/pdo_statistics
```

### Common Error Patterns

**Linker Errors** (`undefined reference to ...`):
- **Cause**: New source file not compiled (CMake caching issue)
- **Fix**: `rm -rf build/ros2_waveshare && colcon build --packages-select ros2_waveshare`

**Message Field Name Mismatches**:
- **Symptom**: `error: 'struct MotorFeedback' has no member named 'position'`
- **Fix**: Check actual message definition in `src/ros2_waveshare_msgs/msg/*.msg`
- **Pattern**: Message uses `position_rad` not `position`, `target_velocity_rad_s` not `target_velocity`

**PDO Callback Not Firing**:
- **Check**: `pdo_manager_->is_running()` returns true
- **Check**: TPDO callbacks registered for correct node_id
- **Debug**: Add `RCLCPP_INFO` at start of `on_tpdo1_received()` to verify it's called

## Message Type Conventions

**MotorCommand** (subscriber input):
- `target_position_rad` (NOT `target_position`)
- `target_velocity_rad_s` (NOT `target_velocity`)
- `target_torque_nm`

**MotorFeedback** (publisher output):
- `position_rad`, `velocity_rad_s`, `torque_nm` (SI units)
- `encoder_counts`, `velocity_counts_per_sec`, `current_ma` (raw values)
- `statusword`, `operation_mode_display`, `state` (CIA402)

**PDOStatistics** (aggregated across motors):
- Single scalar values: `tpdo1_received`, `tpdo2_received`, `rpdo1_sent`, etc.
- NOT arrays - aggregate totals from all motors

## Future Work

**Next Implementation Tasks** (in order):
1. **Enable Motor Action** - CIA402 state machine transitions (Shutdown → Switch On → Enable Operation)
2. **Reset Fault Action** - Clear fault state, verify with `decode_statusword()`
3. **Move to Position Action** - Profile Position mode with progress monitoring via PDO feedback

**Reference**: See waveshare_cpp library's `include/canopen/cia402_fsm.hpp` for state machine implementation.
**Configuration**: All YAML/launch files in `*/config/` and `*/launch/` subdirectories
- `bridge_params.yaml`: Serial baud (2Mbps), CAN baud (500k/1Mbps), interface names
- `bus.yml`: CANOpen node IDs, EDS paths, driver types
- EDS files: Device object dictionaries (don't edit unless hardware spec changes)

**Source Layout**:
- `include/<package>/`: Public headers (installed)
- `src/`: Implementation files (.cpp)
- `scripts/`: Standalone utilities (wave_reader, wave_writer, etc.)
- `doc/`: Architecture diagrams (Mermaid), integration guides

**Git Submodules**: `waveshare_cpp` is a submodule. Update with:
```bash
git submodule update --remote src/ros2_waveshare/lib/waveshare_cpp
# Then commit the submodule pointer change
```

## Common Issues

**Build Failures**:
- Missing ROS2 deps: `rosdep install --from-paths src --ignore-src -r -y`
- Submodule not initialized: `git submodule update --init --recursive`
- Stale CMake cache: `rm -rf build/ install/` then rebuild

**Runtime Errors**:
- "Device not found": Check `/dev/ttyUSB*` exists, verify udev rules
- "Permission denied": Run `sudo chmod 666 /dev/ttyUSB0` or fix udev rules
- "CAN interface not found": Verify `ip link show vcan0` shows interface UP
- Bridge won't activate: Check serial/CAN baud rates in YAML match hardware config

**CANOpen Issues**:
- No motor response: Verify CAN baud rate (1Mbps typical), node IDs (2-5) match physical DIP switches
- PDO not received: Check EDS PDO mappings match motor configuration

## DevContainer Support

**Multi-GPU Configurations**: `.devcontainer/intel-gpu/` and `.devcontainer/nvidia-gpu/`
- Supports Xorg and Wayland
- Use `.devcontainer/setup-devcontainer.sh` to select configuration
- Each config includes ROS2 Humble, CAN tools, hardware acceleration

## Integration with External Systems

**ros2_canopen Stack** (Lely-based):
- Expects SocketCAN interface (bridge provides this)
- Loads EDS files at launch, generates DCF files
- Manages NMT state machine, SDO configuration, PDO subscriptions

**ros2_control** (Future):
- Hardware interface will communicate via CANOpen topics/services
- Control loop runs at ROS2 rate, PDOs provide real-time feedback

**External CAN Tools**: Standard Linux utilities work via SocketCAN:
- `candump`, `cansend`, `cangen`, `cansequence` for testing
- `can-utils` package provides full toolchain