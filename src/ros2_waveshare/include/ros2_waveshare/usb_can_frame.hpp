/**
 * @file usb_can_frame.hpp
 * @brief USB-CAN adapter frame handling for ROS2
 * This file defines structures and functions to handle USB-CAN-A adapter frames,
 * including conversion between abstract CAN frames and adapter-specific formats.
 * The implementation is based on the USB-CAN-A example from Waveshare (canusb.c)
 * and the ros2_canopen package.
 * @author Andrea Efficace
 * @date August 2025
 */

#pragma once

#include "usb_can_common.hpp"

#include <vector>
#include <cstdint>
#include <memory>
#include <array>

using namespace std;

namespace usb_can_bridge
{
// Base frame structure
#pragma pack(push, 1)
struct USBCANAdapterBaseFrame {
	const uint8_t start_byte = static_cast<uint8_t>(USBCANConst::START_BYTE); // Start byte (0xAA)

};
}
