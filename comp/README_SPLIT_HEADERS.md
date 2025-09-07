# USB-CAN Frame Classes - Split Header Structure

This directory contains the class-based implementation of USB-CAN adapter frames, split into separate header files for better organization and maintainability.

## File Structure

### Core Header Files

#### `usb_can_common.hpp`
Common definitions, enums, and utility functions used by all frame classes:
- Constants (START_BYTE, MSG_HEADER, etc.)
- Enums (Type, FrameType, FrameFmt, CANBaud, etc.)
- Utility functions (to_uint8, make_variable_type_byte, etc.)
- Index enums (FixedSizeIndex, VarSizeIndex)

#### `adapter_base_frame.hpp`
Abstract base class defining the common interface for all frame types:
- Pure virtual methods for frame operations
- Basic validation (start byte check)
- Move semantics support
- Copy protection to prevent slicing

#### `fixed_size_frame.hpp`
Implementation of the 20-byte fixed frame format:
- FixedSizeFrame class
- Automatic checksum management
- Data validation and bounds checking
- Serialization/deserialization

#### `variable_size_frame.hpp`
Implementation of the variable length frame format:
- VariableSizeFrame class
- Dynamic sizing based on frame type and data length
- Index management for variable structure
- Serialization/deserialization

#### `frame_factory.hpp`
Factory class for creating frame objects:
- Static methods for frame creation
- Auto-detection of frame type from raw data
- Builder pattern implementation
- Input validation

#### `usb_can_frames.hpp`
Main include file that includes all other headers:
- Single include point for users
- Convenience type aliases
- Version information
- Usage documentation

### Example and Test Files

#### `class_usage_example.cpp`
Complete example demonstrating usage of the class-based implementation:
- Creating fixed and variable frames
- Setting frame properties (ID, data, DLC)
- Serialization and deserialization
- Polymorphic usage
- Error handling examples

#### `test_headers.sh`
Shell script to test compilation of all headers:
- Individual header compilation tests
- Example application compilation
- Runtime testing
- Error reporting

### Documentation Files

#### `STRUCT_VS_CLASS_COMPARISON.md`
Detailed comparison between struct and class implementations:
- Key differences in design
- Advantages and disadvantages
- Migration guide
- Usage examples

## Usage

### Simple Usage
Include the main header file and use the factory to create frames:

```cpp
#include "usb_can_frames.hpp"

using namespace USBCANBridge;

// Create a fixed frame
auto fixed_frame = FrameFactory::createFixedFrame(
    Type::DATA_FIXED, FrameType::STD_FIXED, FrameFmt::DATA_FIXED);

// Set frame properties
fixed_frame->setID(0x123);
fixed_frame->setData({0x01, 0x02, 0x03, 0x04});
fixed_frame->setDLC(4);

// Serialize
auto data = fixed_frame->serialize();
```

### Advanced Usage
Use individual headers when you only need specific functionality:

```cpp
#include "fixed_size_frame.hpp"
#include "frame_factory.hpp"

// Only fixed frames available
auto frame = FrameFactory::createFixedFrame(
    Type::DATA_FIXED, FrameType::STD_FIXED, FrameFmt::DATA_FIXED);
```

### Polymorphic Usage
Use the base class for containers of mixed frame types:

```cpp
#include "usb_can_frames.hpp"

std::vector<std::unique_ptr<AdapterBaseFrame>> frames;
frames.push_back(FrameFactory::createFixedFrame(...));
frames.push_back(FrameFactory::createVariableFrame(...));

for (auto& frame : frames) {
    if (frame->isValidFrame()) {
        auto data = frame->serialize();
        // Process frame...
    }
}
```

## Key Benefits of Split Structure

### 1. **Modularity**
- Include only what you need
- Reduced compilation dependencies
- Easier to understand individual components

### 2. **Maintainability**
- Changes to one class don't affect others
- Clear separation of concerns
- Easier debugging and testing

### 3. **Reusability**
- Individual classes can be used independently
- Factory pattern enables flexible object creation
- Base class allows polymorphic usage

### 4. **Compilation Performance**
- Faster incremental compilation
- Reduced header bloat
- Better dependency management

## Compilation

### Requirements
- C++17 or later
- Standard library support for `<memory>`, `<vector>`, `<array>`

### Compilation Test
Run the test script to verify all headers compile correctly:

```bash
./test_headers.sh
```

### Manual Compilation
Compile the example manually:

```bash
g++ -std=c++17 -Wall -Wextra class_usage_example.cpp -o example
./example
```

## Migration from Struct Version

To migrate from the struct-based implementation:

1. Replace the include:
   ```cpp
   // Old
   #include "usb_can_frame.hpp"
   
   // New
   #include "usb_can_frames.hpp"
   ```

2. Use factory for object creation:
   ```cpp
   // Old
   fixedSize frame(Type::DATA_FIXED, FrameType::STD_FIXED, FrameFmt::DATA_FIXED);
   
   // New
   auto frame = FrameFactory::createFixedFrame(Type::DATA_FIXED, FrameType::STD_FIXED, FrameFmt::DATA_FIXED);
   ```

3. Replace direct member access with methods:
   ```cpp
   // Old
   frame.id_bytes[0] = 0x12;
   
   // New
   frame->setID(0x1234);
   ```

4. Remove manual checksum management:
   ```cpp
   // Old
   frame.checksum = frame.calculateChecksum();
   
   // New
   // Checksum is automatically managed
   ```

## Version Information

- Version: 1.0.0
- Compatible with: C++17 and later
- License: Same as original project

For more detailed information, see the individual header files and the comparison document.
