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
#include <algorithm>
#include <cstring>
#include <stdexcept>

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

	USBCANAdapter20ByteFrame() : USBCANAdapterBaseFrame() {
		// Initialize fixed fields
		data.fill(0);
		checksum = 0;
	}

	const uint8_t calculateChecksum(const USBCANAdapter20ByteFrame& frame){
		/**
		 * @brief Calculate checksum for a 20-byte USB-CAN-A adapter frame
		 * The checksum is the low 8 bits of the cumulative sum from the frame type byte
		 * to the reserved byte (inclusive).
		 * @param frame The 20-byte frame for which to calculate the checksum
		 * @return The calculated checksum byte
		 */

		// Init
		uint32_t sum = 0;
		// Sum bytes from frame type (data[0]) to reserved (data[17])
		sum += frame.data[0]; // Type byte
		sum += std::accumulate(frame.data.begin() + 1, frame.data.end(), 0U);
		// Return low 8 bits
		return static_cast<uint8_t>(sum & 0xFF);

	}

	// STL interface and Operator overloading
	uint8_t& operator[](std::size_t index) {
		if (index == 0) {
			return const_cast<uint8_t&>(start_byte);
		} else if (index == 1) {
			return const_cast<uint8_t&>(msg_header);
		} else if (index >= 2 && index < 19) {
			return data[index - 2];
		} else if (index == 19) {
			return checksum;
		} else {
			throw std::out_of_range("Index out of range for USBCANAdapter20ByteFrame");
		}
	}

	const uint8_t& operator[](std::size_t index) const {
		if (index == 0) {
			return start_byte;
		} else if (index == 1) {
			return msg_header;
		} else if (index >= 2 && index < 19) {
			return data[index - 2];
		} else if (index == 19) {
			return checksum;
		} else {
			throw std::out_of_range("Index out of range for USBCANAdapter20ByteFrame");
		}
	}

	// at() method with bounds checking
	uint8_t& at(std::size_t index) {
		return this->operator[](index);
	}
	const uint8_t& at(std::size_t index) const {
		return this->operator[](index);
	}

	// size() method
	std::size_t size() const {
		return 20;
	}

	// Iterator Interface - for range-based for loops on data field
	auto begin() {
		return data.begin();
	}
	auto end() {
		return data.end();
	}
	auto begin() const {
		return data.begin();
	}
	auto end() const {
		return data.end();
	}

	// fill method to set all data bytes to a specific value
	void fill(uint8_t value) {
		data.fill(value);
	}

	// set specific data byte
	void setData(const std::vector<uint8_t>& new_data) {
		// Ensure new_data has exactly 17 bytes or less
		if (new_data.size() <= data.size()) {
			throw std::invalid_argument("Data size must be 17 bytes or less for USBCANAdapter20ByteFrame");
		}
		// Find the number of bytes to copy
		std::size_t bytes_to_copy = std::min(new_data.size(), data.size());
		// Copy new data into the data array
		std::copy(new_data.begin(), new_data.begin() + bytes_to_copy, data.begin());
		// Zero out any remaining bytes if new_data is smaller than data array
		if (bytes_to_copy < data.size()) {
			std::fill(data.begin() + bytes_to_copy, data.end(), 0);
		}
	}
	// get data as vector
	std::vector<uint8_t> getData() const {
		return std::vector<uint8_t>(data.begin(), data.end());
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

	USBCANAdapterVariableFrame() : USBCANAdapterBaseFrame(),
		end_byte(static_cast<uint8_t>(USBCANConst::MSG_HEADER)) {
		// Initialize type byte to default
		type.fill(0);
		// Set default type byte with bits 7 and 6 set
		type[0] = 0xC0;
	}
	explicit USBCANAdapterVariableFrame(uint8_t frame_type, USBCANFrameFmtVar frame_fmt, uint8_t dlc) : USBCANAdapterBaseFrame(),
		end_byte(static_cast<uint8_t>(USBCANConst::MSG_HEADER)) {
		// Initialize type byte
		type.fill(0);
		// Construct type byte
		type[0] = 0xC0;         // Set bits 7 and 6
		if (frame_type == static_cast<uint8_t>(USBCANFrameFmt::EXT)) {
			type[0] |= 0x10;         // Set bit 5 for extended frame
		}
		if (frame_fmt == USBCANFrameFmtVar::REMOTE) {
			type[0] |= 0x10;         // Set bit 4 for remote frame
		}
		type[0] |= (dlc & 0x0F);         // Set bits 3-0 for DLC
	}

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

	// Safe data manipulation
	void setIDBytes(const std::vector<uint8_t>& id_bytes)
	{
		id = id_bytes;
		if (id.size() != 2 && id.size() != 4) {
			throw std::invalid_argument("ID size must be 2 or 4 bytes for USBCANAdapterVariableFrame");
		}
	}
	void setDataBytes(const std::vector<uint8_t>& data_bytes)
	{
		data = data_bytes;
		if (data.size() > 8) {
			data.resize(8); // CAN data max 8 bytes
		}
	}
	void reserveCapacity(size_t id_size, size_t data_size)
	{
		id.reserve(id_size);
		data.reserve(data_size);
	}
};
#pragma pack(pop)



bool isFrameComplete(const std::vector<uint8_t>& frame);
}
