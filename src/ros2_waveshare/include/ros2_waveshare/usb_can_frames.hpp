/**
 * @file usb_can_frames.hpp
 * @brief Main include file for USB-CAN adapter frame classes
 * This file includes all the USB-CAN frame class headers, providing a single
 * include point for users who want to use the complete class-based USB-CAN
 * frame implementation.
 *
 * The implementation is split into separate header files for better organization:
 * - adapter_base_frame.hpp: Abstract base class for all frames
 * - fixed_size_frame.hpp: Fixed 20-byte frame class
 * - variable_size_frame.hpp: Variable length frame class
 * - frame_factory.hpp: Factory class for creating frame objects
 *
 * Key differences from struct version:
 * - Uses classes with private/protected data members
 * - Provides proper encapsulation with getter/setter methods
 * - Uses RAII principles with proper constructors/destructors
 * - Implements builder pattern for frame creation
 * - Enhanced error handling and validation
 *
 * The inheritance structure:
 * * AdapterBaseFrame (abstract base class for all frames)
 * ├── * FixedSizeFrame (fixed 20-byte frame)
 * └── * VariableSizeFrame (variable length frame)
 *
 * Factory class:
 * * FrameFactory (creates frame objects with validation)
 *
 * @author Andrea Efficace
 * @date September 2025
 */

#pragma once

// Include common definitions first
#include "usb_can_common.hpp"

// Include class definitions in dependency order
#include "adapter_base_frame.hpp"
#include "fixed_size_frame.hpp"
#include "variable_size_frame.hpp"
#include "frame_factory.hpp"

/**
 * @namespace USBCANBridge
 * @brief Namespace containing all USB-CAN adapter frame classes and utilities
 *
 * This namespace provides a complete class-based implementation for handling
 * USB-CAN-A adapter frames, including:
 *
 * - AdapterBaseFrame: Abstract base class defining the interface
 * - FixedSizeFrame: 20-byte fixed frame implementation
 * - VariableSizeFrame: Variable length frame implementation
 * - FrameFactory: Factory for creating frame objects
 *
 * Example usage:
 * @code
 * using namespace USBCANBridge;
 *
 * // Create a fixed frame
 * auto fixed_frame = FrameFactory::createFixedFrame(
 *     Type::DATA_FIXED, FrameType::STD_FIXED, FrameFmt::DATA_FIXED);
 *
 * // Set frame properties
 * fixed_frame->setID(0x123);
 * fixed_frame->setData({0x01, 0x02, 0x03, 0x04});
 * fixed_frame->setDLC(4);
 *
 * // Serialize frame
 * auto serialized = fixed_frame->serialize();
 *
 * // Create a variable frame
 * auto var_frame = FrameFactory::createVariableFrame(
 *     FrameType::STD_VAR, FrameFmt::DATA_VAR, 6);
 *
 * // Auto-detect frame type from data
 * std::vector<uint8_t> raw_data = {0xAA, 0xC6, 0x34, 0x12};
 * auto frame = FrameFactory::createFrameFromData(raw_data);
 * @endcode
 */

// Convenience aliases for easier usage
namespace USBCANBridge
{
// Type aliases for easier usage
using BaseFrame = AdapterBaseFrame;
using FixedFrame = FixedSizeFrame;
using VariableFrame = VariableSizeFrame;
using Factory = FrameFactory;
}

// Version information
#define USB_CAN_FRAMES_VERSION_MAJOR 1
#define USB_CAN_FRAMES_VERSION_MINOR 0
#define USB_CAN_FRAMES_VERSION_PATCH 0

#define USB_CAN_FRAMES_VERSION_STRING "1.0.0"
