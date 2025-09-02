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
#include <numeric>


namespace usb_can_bridge
{
#pragma pack(push, 1)
// Base frame structure
struct USBCANAdapterBaseFrame {
	/**
	 * @brief USB-CAN-A adapter base frame structure
	 * This structure represents the common fields of both fixed and variable length
	 * frame formats used by the USB-CAN-A adapter. It includes the start byte and
	 * a method to validate the start byte.
	 */
	const uint8_t start_byte = static_cast<uint8_t>(USBCANConst::START_BYTE); // Start byte (0xAA)

	// basic check to verify start byte
	bool isValidStart() const
	{
		return start_byte == static_cast<uint8_t>(USBCANConst::START_BYTE);
	}
};
// Fixed Frame structure (20 bytes)
struct USBCANAdapter20ByteFrame : public USBCANAdapterBaseFrame {
	/**
	 * @brief USB-CAN-A adapter 20-byte fixed frame structure
	 * This structure represents the 20-byte fixed frame format used by the USB-CAN-A
	 * adapter. It includes fields for the CAN ID, data length code (DLC),
	 * data bytes, reserved byte, and checksum.
	 * The complete frame consists of:
	 * - Start byte (always {0xAA}) : inherited from base
	 * - Message header byte (always {0x55}) : inherited from base
	 * - Type byte (always {0x01})
	 * - Frame type byte ({0x01} for standard, {0x02} for extended)
	 * - Framework Format byte ({0x01} for data frame, {0x02} for remote frame)
	 * - Frame ID [4] bytes, high bytes at the front, low bytes at the end (little-endian)
	 * - Frame Data Length Code (DLC) byte (0-8)
	 * - Frame Data [8] bytes (padded with zeros if DLC < 8)
	 * - Reserved byte (always {0x00})
	 * - Checksum Code (The low 8 bits of the cumulative sum from frame type to error code )
	 * @note This fixed frame format is less flexible than the variable length format
	 */
	// Data bytes (17 bytes: type, frame type, ID[4], DLC, Data[8], reserved, checksum)
	const uint8_t msg_header = static_cast<uint8_t>(USBCANConst::MSG_HEADER); // Message header byte
	std::array<uint8_t, 17> data;
	// Checksum byte
	uint8_t checksum;

	const uint8_t calculateChecksum(const USBCANAdapter20ByteFrame& frame){
		/**
		 * @brief Calculate checksum for a 20-byte USB-CAN-A adapter frame
		 * The checksum is the low 8 bits of the cumulative sum from the frame type byte
		 * to the reserved byte (inclusive).
		 * @param frame The 20-byte frame for which to calculate the checksum
		 * @return The calculated checksum byte
		 */

		// Init
		uint32_t checksum = 0;
		// Sum bytes from frame type (data[0]) to reserved (data[17])
		checksum += frame.data[0]; // Type byte
		checksum += std::accumulate(frame.data.begin() + 1, frame.data.end(), 0U);
		// Return low 8 bits
		return static_cast<uint8_t>(checksum & 0xFF);

	}
};
// Variable Length Frame structure
struct USBCANAdapterVariableFrame : public USBCANAdapterBaseFrame
{
	/**
	 * @brief USB-CAN-A adapter variable length frame structure
	 * This structure represents the variable length frame format used by the USB-CAN-A
	 * adapter. It includes fields for the CAN ID, data length code (DLC),
	 * data bytes, and end byte.
	 * The complete frame consists of:
	 * - Start byte (always {0xAA}) : inherited from base
	 * - Type byte:
	 * 	- [Type header] bit 6-7: Always 0xC0 (bits 7 and 6 set to 1)
	 * 	- [Frame Type] bit 5: 0 for standard frame (2 bytes ID), 1 for extended frame (4 bytes ID)
	 * 	- [Frame Format] bit 4: 0 for data frame, 1 for remote frame
	 * 	- [DLC] bits 3-0: Data Length Code (0-8)
	 * - Frame ID [2 or 4] bytes, little-endian
	 * - Frame Data [0-8] bytes
	 * - End byte (always {0x55})
	 * @note This variable length frame format is more flexible and is used as default in this implementation
	 */
	std::array<uint8_t, 1> type; // Type byte
	std::vector<uint8_t> id;   // ID bytes (2 or 4 bytes, little-endian)
	std::vector<uint8_t> data; // Data bytes (0-8 bytes)
	const uint8_t end_byte = static_cast<uint8_t>(USBCANConst::END_BYTE); // End byte (0x55)

	// basic check to verify end byte
	bool isValidEnd() const
	{
		return end_byte == static_cast<uint8_t>(USBCANConst::END_BYTE);
	}
	// basic check to verify frame length
	bool isValidLength() const
	{
		// Extract DLC from type byte (lower 4 bits)
		uint8_t dlc = type[0] & 0x0F;
		// Check if data length matches DLC
		return data.size() == dlc;
	}
};
#pragma pack(pop)



bool isFrameComplete(const std::vector<uint8_t>& frame);
}
