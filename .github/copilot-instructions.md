# ROS2 CAN Bus Control System - AI Coding Instructions

## Project Overview

A ROS2-based CAN bus control middleware using Waveshare USB-CAN-A adapters for industrial motor control. The system bridges USB-CAN hardware to Linux SocketCAN, integrating with CANOpen protocols for servo drive control via CIA402 profiles.

**Current State**: Waveshare C++ library complete with SocketCAN bridge, ROS2 lifecycle nodes in development, targeting CI402 servo drive integration for position/velocity/torque control.

## System Architecture

The system follows a layered architecture:

```
Application Layer (ROS2) → CANOpen Stack → SocketCAN Bridge → Waveshare USB-CAN-A Hardware
```

### Core Components

1. **Waveshare C++ Library** (`src/ros2_waveshare/lib/waveshare_cpp/`): Type-safe C++ library implementing State-First Architecture with CRTP patterns
2. **ROS2 Lifecycle Bridge** (`src/ros2_waveshare/`): ROS2 nodes managing bridge lifecycle 
3. **CANOpen Configuration** (`src/microphase_can_config/`): Device configuration for MICROPHASE SRL TRAC_PWR drives
4. **Speed Control** (`src/speed_control/`): High-level motor control logic with PDO/SDO handling

## Critical Development Patterns

### 1. ROS2 Workspace Management

**Build System**: Uses colcon with symlink installs
```bash
# Always build from workspace root
cd /home/ubuntu/ros_ws
colcon build --symlink-install

# Source setup after builds
source install/setup.bash
```

**Environment Setup**: Use provided scripts for consistent environment:
- `scripts/setup_ros2_environment.sh`: Configures ROS2 environment 
- `scripts/build_ws.sh`: Automated dependency resolution and build
- Environment variables: `ROS_DISTRO`, `WORKSPACE=/home/ubuntu/ros_ws`

### 2. Waveshare Library Integration

**Location**: `src/ros2_waveshare/lib/waveshare_cpp/` (git submodule)
**Key Pattern**: State-First Architecture - frames are stateful objects generating protocol buffers on-demand

```cpp
// State-first frame usage
FixedFrame frame;
frame.set_id(0x123);              // Modifies internal state
frame.set_data({0x11, 0x22});     // No direct buffer manipulation
auto buffer = frame.serialize();   // Generates protocol buffer on-demand
```

**Thread Safety**: `USBAdapter` uses `state_mutex_` (shared_mutex) for configuration, separate mutexes for I/O operations

### 3. CAN Bus Workflow

**Device Setup**: USB-CAN-A adapters appear as `/dev/ttyUSB*` devices requiring udev rules:
```bash
# Required udev rule in /etc/udev/rules.d/50-myusb.rules
KERNEL=="ttyUSB[0-9]*",MODE="0666"
```

**SocketCAN Bridge**: Essential for ROS2 CANOpen integration:
- Virtual interfaces: Create `vcan0` for testing, real interfaces for hardware
- Bridge translates between Waveshare protocol and SocketCAN frames bidirectionally

**Module Loading**: Required kernel modules for CAN operations:
```bash
sudo modprobe can can-raw can-bcm vcan
sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
```

### 4. CANOpen Device Integration

**Target Hardware**: MICROPHASE SRL TRAC_PWR drives (Vendor ID: 0x1A21, Product: 4)
**Protocol**: DS-402 motion control profile with standard objects:
- `0x6040`: Control Word (16-bit, PDO mappable)
- `0x6041`: Status Word (16-bit, PDO mappable) 
- `0x6060/6061`: Mode of Operation (display)
- `0x607A`: Target Position, `0x60FF`: Target Velocity
- `0x606C`: Velocity Actual, `0x6064`: Position Actual

**Configuration**: EDS files in `src/microphase_can_config/config/` define device parameters and PDO mappings

### 5. Package Dependencies & Build

**Key ROS2 Dependencies**:
- `canopen_core`, `canopen_interfaces`: CANOpen stack integration
- `lely_core_libraries`: Low-level CANOpen implementation
- `rclcpp_lifecycle`: Lifecycle node management
- `diagnostic_updater`: System health monitoring

**Build Order**: Dependencies auto-resolved by `rosdep`, but understand:
1. Waveshare C++ library (standalone CMake)
2. Core ROS2 packages (microphase_can_config, ros2_waveshare)
3. Application packages (speed_control)

## Development Workflows

### Testing Strategy

**Waveshare Library**: 132 tests, hardware-independent via dependency injection
```bash
cd src/ros2_waveshare/lib/waveshare_cpp/
cmake -B build && cmake --build build && ctest --test-dir build
```

**ROS2 Testing**: Use `colcon test` for package-level testing
```bash
colcon test --packages-select ros2_waveshare speed_control
colcon test-result --verbose  # View detailed results
```

### Hardware Debugging

**USB-CAN Adapter**: Use `wave_reader` script for low-level debugging:
```bash
# From waveshare_cpp build directory
./build/scripts/wave_reader /dev/ttyUSB0
```

**SocketCAN Monitoring**: Standard Linux CAN utilities:
```bash
candump vcan0                    # Monitor CAN traffic
cansend vcan0 123#DEADBEEF      # Send test frames (CAN ID 0x123, data DEADBEEF)
```

### Lifecycle Management

**ROS2 Nodes**: Use lifecycle patterns for graceful startup/shutdown:
- Configure → Activate → Deactivate → Cleanup states
- Bridge nodes manage USB adapter connection lifecycle
- Diagnostic publishing for system health monitoring

## File Organization Conventions

### Configuration Management
- **Launch files**: `*/launch/*.launch.py` with parameterized configurations
- **Parameters**: YAML files in `*/config/` directories
- **Device configs**: EDS files for CANOpen device descriptions

### Source Structure
- **Headers**: Public interfaces in `include/*/` 
- **Implementation**: Source files in `src/`
- **Scripts**: Utilities and tools in `scripts/` (executable)
- **Documentation**: Architecture docs in `doc/`, README files at package level

### Git Workflow
- **Branch**: Currently on `microphase` branch for servo drive integration
- **Submodules**: Waveshare library is a git submodule, update carefully
- **CI/CD**: GitHub workflows for automated testing (when available)

## Integration Points

### Cross-Package Communication
- **CAN frames**: Standard SocketCAN interface between all ROS2 nodes
- **Control commands**: `geometry_msgs/Twist` and custom message types
- **Status feedback**: Motor status via `diagnostic_msgs` and custom status messages
- **Service calls**: Configuration and emergency stop via ROS2 services

### External Dependencies
- **Linux kernel**: CAN socket support, USB serial drivers (ch341-uart)
- **Hardware**: Waveshare USB-CAN-A adapters, MICROPHASE servo drives
- **CANOpen stack**: Lely libraries for protocol implementation

## Common Issues & Solutions

### Build Failures
- **Missing dependencies**: Run `rosdep install --from-paths src --ignore-src -r -y`
- **Submodule issues**: Ensure `git submodule update --init --recursive`
- **CMake cache**: Clear `build/` directory for clean builds

### Runtime Issues  
- **Device permissions**: Check udev rules for `/dev/ttyUSB*` access
- **CAN modules**: Verify kernel modules loaded with `lsmod | grep can`
- **SocketCAN interfaces**: Use `ip link show` to verify interface state

Focus on the bridge architecture - the Waveshare library provides the low-level CAN communication, while ROS2 packages handle the application logic and CANOpen protocol integration.