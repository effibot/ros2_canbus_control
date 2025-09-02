# USB-CAN Frame System

This directory contains a complete implementation of the USB-CAN-A adapter frame system using modern C++ design patterns.

## Architecture Overview

### Design Patterns Used

1. **Factory Pattern**: For creating frame objects of different types
2. **Builder Pattern**: For constructing complex frames with method chaining
3. **Inheritance Hierarchy**: For code reuse and polymorphism
4. **RAII**: For automatic resource management

### Frame Type Hierarchy

```
USBCANFrame (Abstract Base)
├── USBCANFrame20Byte (20-byte frames)
│   ├── USBCANSettingsFrame
│   ├── USBCANStandardFixed20Frame
│   └── USBCANExtendedFixed20Frame
└── USBCANVariableFrame (Variable length frames)
    ├── USBCANStandardVariableFrame
    └── USBCANExtendedVariableFrame
```

## File Organization

### Header Files (`include/ros2_waveshare/frames/`)

- `usb_can_frame_base.hpp` - Base structures and abstract frame class
- `usb_can_frame_20byte.hpp` - Base class for 20-byte frames
- `usb_can_frame_variable.hpp` - Base class for variable frames
- `usb_can_settings_frame.hpp` - Settings frame implementation
- `usb_can_fixed_frames.hpp` - Fixed 20-byte CAN frame implementations
- `usb_can_variable_frames.hpp` - Variable length CAN frame implementations
- `usb_can_frame_factory.hpp` - Factory pattern implementation
- `usb_can_frame_builder.hpp` - Builder pattern implementation
- `usb_can_frames.hpp` - Main include file (includes everything)

### Source Files (`src/frames/`)

- `usb_can_frame_examples.cpp` - Complete usage examples

### Test Files (`test/`)

- `test_frames.cpp` - Comprehensive unit tests

## Usage Examples

### 1. Using Factory Pattern

```cpp
#include "ros2_waveshare/frames/usb_can_frames.hpp"

usb_can_bridge::FrameFactory factory;

// Create different frame types
auto settings = factory.createFrame(USBCANFrameType::SETTING_FRAME);
auto can_frame = factory.createFrame(USBCANFrameType::STANDARD_VARIABLE);

// Create from raw data (auto-detects type)
std::vector<uint8_t> raw_data = { /* received data */ };
auto received_frame = factory.createFrameFromData(raw_data);
```

### 2. Using Builder Pattern

```cpp
// Build a standard CAN frame
auto frame = usb_can_bridge::FrameBuilder()
    .setFrameType(USBCANFrameType::STANDARD_VARIABLE)
    .setCANID(0x123)
    .setCANData({0x01, 0x02, 0x03, 0x04})
    .setRTR(false)
    .build();

// Build a settings frame
auto settings = usb_can_bridge::FrameBuilder()
    .setFrameType(USBCANFrameType::SETTING_FRAME)
    .setBaudRate(USBCANBaud::SPEED_500K)
    .setCANMode(USBCANMode::NORMAL)
    .setAutoRetransmit(true)
    .build();
```

### 3. Using Convenience Functions

```cpp
// Quick frame creation
auto std_frame = usb_can_bridge::FrameBuilders::createStandardCANFrame(0x456, {0x10, 0x20});
auto ext_frame = usb_can_bridge::FrameBuilders::createExtendedCANFrame(0x12345678, {0xAA, 0xBB});
auto rtr_frame = usb_can_bridge::FrameBuilders::createRTRFrame(0x789, false);
auto settings = usb_can_bridge::FrameBuilders::createSettingsFrame();
```

### 4. Working with Specific Frame Types

```cpp
// Settings frame configuration
usb_can_bridge::SettingsFrame settings;
settings.setBaudRate(USBCANBaud::SPEED_1M);
settings.setFilterID(0x12345678);
settings.setMaskID(0xFFFFFFFF);

// Standard variable frame
usb_can_bridge::StandardVariableFrame std_frame;
std_frame.setCANID(0x123);
std_frame.setCANData({0x01, 0x02, 0x03});
std_frame.setRTR(false);

// Extended fixed frame
usb_can_bridge::ExtendedFixed20Frame ext_frame;
ext_frame.setCANID(0x1ABCDEF0);
ext_frame.setCANData({0xDE, 0xAD, 0xBE, 0xEF});
ext_frame.setTimestamp(getCurrentTime());
```

### 5. Serialization and Deserialization

```cpp
// Serialize frame to bytes
auto frame_data = frame->serialize();

// Send over serial port
send_to_serial(frame_data);

// Deserialize received data
std::vector<uint8_t> received_data = read_from_serial();
auto received_frame = factory.createFrameFromData(received_data);

if (received_frame && received_frame->isValid()) {
    // Process frame
    switch (received_frame->getType()) {
        case USBCANFrameType::STANDARD_VARIABLE:
            // Handle standard CAN frame
            break;
        case USBCANFrameType::SETTING_FRAME:
            // Handle settings response
            break;
        // ... other cases
    }
}
```

## Key Features

### 1. Type Safety
- STL interfaces with bounds checking
- Template-based compile-time size validation
- Exception handling for invalid operations

### 2. Memory Efficiency
- `std::array` for fixed-size data (stack allocated)
- `std::vector` only where flexibility is needed
- Minimal dynamic allocation in critical paths

### 3. STL Integration
- Range-based for loops support
- STL algorithm compatibility
- Iterator interfaces

### 4. Error Handling
- Exception-based error reporting
- Checksum validation for 20-byte frames
- Input validation and sanitization

### 5. Performance
- Zero-copy operations where possible
- Efficient serialization/deserialization
- Cache-friendly memory layout

## Building and Testing

### Build the Library

```bash
cd /home/ubuntu/ros_ws
colcon build --packages-select ros2_waveshare
```

### Run Examples

```bash
./build/ros2_waveshare/frame_examples
```

### Run Tests

```bash
colcon test --packages-select ros2_waveshare
```

## Integration with ROS2

The frame system is designed to integrate seamlessly with ROS2:

- Use with `rclcpp::Logger` for consistent logging
- Compatible with ROS2 message types
- Thread-safe for use in ROS2 nodes
- Exception-safe for ROS2 lifecycle management

## Protocol Compliance

The implementation follows the USB-CAN-A protocol specification:

- Correct frame formats for all message types
- Proper checksum calculation for 20-byte frames
- Support for both variable and fixed frame lengths
- Standard and extended CAN ID support
- RTR (Remote Transmission Request) frame support

## Future Extensions

The design supports easy extension for:

- Additional frame types
- New protocol versions
- Enhanced filtering capabilities
- Performance optimizations
- Additional validation features

## Memory Safety

All code uses modern C++ safety features:

- RAII for automatic resource management
- Smart pointers for memory safety
- Bounds checking for array access
- Exception safety guarantees
