# Copilot Instructions for ROS2 CAN Bus Control

## Project Overview

This is a ROS2 workspace focused on Waveshare USB-CAN-A adapter integration with motor control systems. The project consists of two main architectural layers:

1. **Core USB-CAN Library** (`waveshare_cpp/`) - Modern C++17 library using CRTP patterns
2. **ROS2 Integration Packages** (`src/`) - ROS2 nodes and bridges for real-world deployment

## Key Architecture Patterns

### CRTP-Based Frame System
The core library uses Curiously Recurring Template Pattern for zero-runtime-overhead polymorphism:

```cpp
// waveshare_cpp/include/base_frame.hpp
template<typename Derived>
class BaseFrame {
    Derived& derived() { return static_cast<Derived&>(*this); }
    // Delegates to derived implementation via CRTP
};

// Concrete implementations
class FixedFrame : public BaseFrame<FixedFrame> { /* 20-byte frames */ };
class ConfigFrame : public BaseFrame<ConfigFrame> { /* Device config */ };
```

### USB-CAN Protocol Implementation
Three frame types implement Waveshare's custom protocol:
- **Fixed Frames** (20 bytes): Standard CAN data/config with padding
- **Variable Frames**: Dynamic size based on actual CAN payload 
- **Config Frames**: Device setup (baud rates, filters, modes)

Protocol constants in `common.hpp`:
- `START_BYTE = 0xAA`, `MSG_HEADER = 0x55`, `END_BYTE = 0x55`
- Frame types: `DATA_FIXED = 0x01`, `CONF_VAR = 0x12`, `DATA_VAR = 0xC0`

## Build System & Workflows

### Standard Build Commands
```bash
# Full workspace build
./scripts/build_ws.sh  # Handles rosdep + colcon build

# Individual package builds
colcon build --packages-select ros2_waveshare
colcon build --packages-select usb_can_bridge_test --cmake-clean-first
```

### Testing Framework
The `usb_can_bridge_test` package provides hardware-in-the-loop testing:
- Requires 2x Waveshare USB-CAN-A adapters (`/dev/ttyUSB0`, `/dev/ttyUSB1`)
- Virtual motor simulator for CANopen protocol testing
- SocketCAN bridge for `vcan0` integration

## Code Organization Conventions

### Namespace Structure
All code lives under `USBCANBridge` namespace:
```cpp
namespace USBCANBridge {
    // Core protocol types
    enum class Type : uint8_t { DATA_FIXED = 0x01, CONF_VAR = 0x12 };
    enum class CANBaud : uint8_t { SPEED_1000K = 0x01, SPEED_500K = 0x02 };
    
    // Utility functions
    template<typename EnumType> 
    constexpr std::byte to_byte(EnumType value);
}
```

### File Naming Patterns
- Headers: `snake_case.hpp` (modern C++ style)
- Implementation: `snake_case.cpp` 
- ROS2 nodes: `node_name.cpp` in `src/package_name/src/`

### Error Handling Strategy
Uses `Result<T>` monad pattern instead of exceptions:
```cpp
Result<Status> ConfigFrame::set_baud_rate(CANBaud baud) {
    if (!is_valid_baud(baud)) {
        return Result<Status>::error(Error::INVALID_BAUD_RATE);
    }
    // ... implementation
    return Result<Status>::success(Status::SUCCESS);
}
```

## Development Environment

### Container Setup
- Ubuntu 24.04.3 LTS devcontainer
- ROS2 Jazzy distribution
- VS Code with Remote-Containers extension
- Hardware access via `/dev/ttyUSB*` device passthrough

### Key Dependencies
- `libusb-1.0` for USB device communication
- `can-utils` for SocketCAN interface testing
- Standard ROS2 packages: `rclcpp`, `std_msgs`, `geometry_msgs`

## Integration Points

### ROS2 Node Architecture
- **USB-CAN Driver Node**: Low-level hardware interface
- **Traction Controller Node**: High-level motor control commands
- **Bridge Nodes**: Protocol conversion (USB-CAN ↔ SocketCAN)

### Message Flow
```
geometry_msgs/Twist → CAN Protocol Frames → USB Serial → Hardware
Hardware → USB Serial → CAN Protocol Frames → Motor Status Messages
```

## Documentation References

- USB-CAN protocol details: `waveshare_cpp/README.md`
- ROS2 package design: `ROS2_USB_CAN_Package_Design.md`
- Testing procedures: `USB_CAN_Bridge_Test_Jazzy.md`
- Refactoring patterns: `comp/USB_CAN_REFACTORING_GUIDE.md`

When working with this codebase:
1. Use the CRTP pattern for new frame types
2. Follow the `Result<T>` error handling convention
3. Test with hardware-in-the-loop setup when possible
4. Maintain the clear separation between core library and ROS2 integration