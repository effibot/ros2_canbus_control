/**
 * @file usb_can_frame.hpp
 * @brief USB-CAN adapter frame handling for ROS2
 * This file define		virtual FrameType getType() const = 0; structures, classes and functions to handle USB-CAN-A adapter frames,
 * to implement the Waveshare USB-CAN-A adapter protocol.
 * The implementation supports both fixed 20-byte frames and variable length frames, with
 * standard and extended CAN IDs.
 * The implementation is based on the USB-CAN-A example from Waveshare (canusb.c) and
 * adapts it to OOP principles and modern C++ practices.
 * The final goal is to provide a robust and flexible interface for CAN communication
 * in ROS2 applications, with safety checks and clear abstractions.
 * The inheritance structure will be as follows:
 * * AdapterBaseFrame (abstract base class for all frames)
 * ├── * Adapter20ByteFrame (fixed 20-byte frame)
 * │   ├── STD20ByteFrame (standard ID frame)
 * │   ├── EXT20ByteFrame (extended ID frame)
 * │   └── SettingsFrame (settings frame)
 * └── * AdapterVariableFrame (variable length frame)
 *     ├── STDVariableFrame (standard ID frame)
 *     └── EXTVariableFrame (extended ID frame)
 * Each frame type will implement methods for serialization, deserialization,
 * validation, and data access, with appropriate safety checks.
 * The code will use STL containers and algorithms for efficiency and clarity.
 *
 * @note This implementation is designed to be used with a Builder pattern for frame creation.
 * @note Error handling is done via exceptions for robustness.
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

namespace USBCANBridge
{
#pragma pack(push, 1)
// Base frame structure
struct baseFrame {
	/**
	 * @brief USB-CAN-A adapter base frame structure
	 * This structure represents the common fields of both fixed and variable length
	 * frame formats used by the USB-CAN-A adapter.
	 * It includes the start byte and a method to validate the start byte.
	 * Here, we also define constructors for the base frame and the operator overloads
	 * that the derived classes must implement.
	 * @note The start byte is always 0xAA for both frame types.
	 */
	const uint8_t start_byte = to_uint8(Constants::START_BYTE); // Start byte (0xAA)

	// basic check to verify start byte
	bool isValidStart() const {
		return start_byte == to_uint8(Constants::START_BYTE);
	}

	// Define the abstract base frame constructors
	baseFrame() : start_byte(to_uint8(Constants::START_BYTE)) {
	}

	// Operators
	virtual uint8_t& operator[](std::size_t index) = 0;
	virtual const uint8_t& operator[](std::size_t index) const = 0;
	virtual uint8_t& at(std::size_t index) = 0;
	virtual const uint8_t& at(std::size_t index) const = 0;

	// Interface for Internal data manipulation
	virtual uint8_t* begin() = 0;
	virtual uint8_t* end() = 0;
	virtual const uint8_t* begin() const = 0;
	virtual const uint8_t* end() const = 0;
	virtual void fill(uint8_t value) = 0;
	virtual std::size_t size() const = 0;
	virtual void setData(const std::vector<uint8_t>& new_data) = 0;
	virtual void setData(size_t index, uint8_t value) = 0;
	virtual std::vector<uint8_t> getData() const = 0;
	virtual uint8_t getDataAt(size_t index) const = 0;

	// Protocol Handling
	virtual void setFrameType(uint8_t frame_type) = 0;
	virtual uint8_t getFrameType() const = 0;
	virtual void setDLC(uint8_t dlc) = 0;
	virtual uint8_t getDLC() const = 0;
	virtual bool isValidLength() const = 0;
	virtual void setID(uint32_t id) = 0;
	virtual uint32_t getID() const = 0;
	virtual std::vector<uint8_t> getIDBytes() const = 0;
	virtual void setIDBytes(const std::vector<uint8_t>& id_bytes) = 0;
	virtual bool isValidID() const = 0;
	virtual std::vector<uint8_t> serialize() const = 0;
	virtual bool deserialize(const std::vector<uint8_t>& data) = 0;
	virtual FrameType getType() const = 0;
	virtual bool isValidFrame() const = 0;
};

// Fixed Frame structure (20 bytes)
struct fixedSize : public baseFrame {
	/**
	 * @brief USB-CAN-A adapter 20-byte fixed frame structure
	 * This structure represents the 20-byte fixed frame format used by the USB-CAN-A
	 * adapter. It includes fields for the CAN ID, data length code (DLC),
	 * data bytes, reserved byte, and checksum.
	 * The complete frame consists of:
	 * - [0] Start byte (always {0xAA}) : inherited from base
	 * - [1] Message header byte (always {0x55}) : inherited from base
	 * - [2] Type byte (always {0x01})
	 * - [3] Frame type byte ({0x01} for standard, {0x02} for extended)
	 * - [4] Framework Format byte ({0x01} for data frame, {0x02} for remote frame)
	 * - [5-8] Frame ID [4] bytes, high bytes at the front, low bytes at the end (little-endian)
	 * - [9] Frame Data Length Code (DLC) byte (0-8)
	 * - [10-17] Frame Data [8] bytes (padded with zeros if DLC < 8)
	 * - [18] Reserved byte (always {0x00})
	 * - [19] Checksum Code (The low 8 bits of the cumulative sum from frame type to error code )
	 * @note This fixed frame format is less flexible than the variable length format
	 */
	// Data bytes (17 bytes: type, frame type, ID[4], DLC, Data[8], reserved, checksum)
	const uint8_t msg_header = to_uint8(Constants::MSG_HEADER); // Message header byte
	std::array<uint8_t, 17> data;
	// Checksum byte
	uint8_t checksum;

	fixedSize() : baseFrame() {
		// Initialize fixed fields
		data.fill(0);
		// Initialize checksum
		checksum = 0;
	}

	~fixedSize() = default;

	// Override operators
	uint8_t& operator[](std::size_t index) override {
		if (index == 0) {
			return const_cast<uint8_t&>(start_byte);
		} else if (index == 1) {
			return const_cast<uint8_t&>(msg_header);
		} else if (index >= 2 && index < 19) {
			return data[index - 2];
		} else if (index == 19) {
			return checksum;
		} else {
			throw std::out_of_range("Index out of range for fixedSize");
		}
	}
	const uint8_t& operator[](std::size_t index) const override {
		return const_cast<fixedSize*>(this)->operator[](index);
	}
	uint8_t& at(std::size_t index) override {
		return this->operator[](index);
	}
	const uint8_t& at(std::size_t index) const override {
		return this->operator[](index);
	}

	// STL interface
	uint8_t* begin() override {
		return data.begin();
	}
	uint8_t* end() override {
		return data.end();
	}
	const uint8_t* begin() const override {
		return data.begin();
	}
	const uint8_t* end() const override {
		return data.end();
	}
	void fill(uint8_t value) override {
		data.fill(value);
	}
	std::size_t size() const override {
		return 20;
	}
	void setData(const std::vector<uint8_t>& new_data) override {
		/**
		 * @brief Set the data bytes of the frame
		 * This method allows setting the data bytes of the frame.
		 * It ensures that the data size does not exceed 17 bytes.
		 * @param new_data The vector of data bytes to set
		 * @throws std::invalid_argument if new_data size exceeds 17 bytes
		 * @note If new_data is smaller than 17 bytes, the remaining bytes are zeroed out
		 */

		if (new_data.size() > data.size()) {
			const std::string msg = "Data size is " + std::to_string(new_data.size()) + ", but must be 17 bytes or less for fixedSize";
			throw std::invalid_argument(msg);
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
	std::vector<uint8_t> getData() const override {
		/**
		 * @brief Get the data bytes of the frame
		 * This method returns the data bytes of the frame as a vector.
		 * It provides a safe way to access the internal data array.
		 * @return A vector containing the data bytes
		 */
		return std::vector<uint8_t>(data.begin(), data.end());
	}
	void setData(size_t index, uint8_t value) override {
		/**
		 * @brief Set a specific data byte by index
		 * This method allows setting a specific byte in the data array.
		 * It performs bounds checking to ensure the index is valid, but does not
		 * check the value being set. This means that any byte value (0-255) can be set.
		 * @param index The index of the byte to set (0-16)
		 * @param value The value to set at the specified index
		 * @throws std::out_of_range if index is out of bounds
		 * @note The index is zero-based, so valid indices are 0 to 16
		 * @note Call this function only if you are sure about the value being set
		 */
		data.at(index) = value;
	}
	uint8_t getDataAt(size_t index) const override {
		/**
		 * @brief Get a specific data byte by index
		 * This method allows retrieving a specific byte from the data array.
		 * It performs bounds checking to ensure the index is valid.
		 * @param index The index of the byte to retrieve (0-16)
		 * @return The value of the byte at the specified index
		 * @throws std::out_of_range if index is out of bounds
		 * @note The index is zero-based, so valid indices are 0 to 16
		 * @note This function provides safe access to the internal data array
		 * @return The byte value at the specified index
		 */
		return data.at(index);
	}

	// Protocol Handling
	void setFrameType(uint8_t frame_type) override {
		/**
		 * @brief Set the frame type in the data array
		 * The frame type is stored in data[1] and can be either 0x01 (standard) or 0x02 (extended)
		 * @param frame_type The frame type byte to set
		 * @throws std::invalid_argument if frame_type is not 0x01 or 0x02
		 */
		if (frame_type != to_uint8(FrameType::STD_FIXED) &&
		    frame_type != to_uint8(FrameType::EXT_FIXED)) {
			throw std::invalid_argument("Invalid frame type. Must be 0x01 (standard) or 0x02 (extended)");
		}
		data[1] = frame_type;
	}
	uint8_t getFrameType() const override {
		/**
		 * @brief Get the frame type from the data array
		 * Frame type is stored in data[0] and can be either 0x01 (standard) or 0x02 (extended)
		 * @return Frame type byte
		 */
		return data[1];
	}
	void setDLC(uint8_t dlc) override {
		if (dlc > 8) {
			throw std::invalid_argument("DLC must be 0-8 for CAN frames");
		}
		data[to_uint8(FixedSizeIndex::DLC)] = dlc;
	}
	uint8_t getDLC() const override {
		return data[static_cast<size_t>(FixedSizeIndex::DLC)];
	}
	void setID(uint32_t id_value) override {
		// Store ID in little-endian format in bytes 3-6
		data[static_cast<size_t>(FixedSizeIndex::ID_0)] = static_cast<uint8_t>(id_value & 0xFF);
		data[static_cast<size_t>(FixedSizeIndex::ID_1)] = static_cast<uint8_t>((id_value >> 8) & 0xFF);
		data[static_cast<size_t>(FixedSizeIndex::ID_2)] = static_cast<uint8_t>((id_value >> 16) & 0xFF);
		data[static_cast<size_t>(FixedSizeIndex::ID_3)] = static_cast<uint8_t>((id_value >> 24) & 0xFF);
	}
	uint32_t getID() const override {
		return static_cast<uint32_t>(data[static_cast<size_t>(FixedSizeIndex::ID_0)]) |
		       (static_cast<uint32_t>(data[static_cast<size_t>(FixedSizeIndex::ID_1)]) << 8) |
		       (static_cast<uint32_t>(data[static_cast<size_t>(FixedSizeIndex::ID_2)]) << 16) |
		       (static_cast<uint32_t>(data[static_cast<size_t>(FixedSizeIndex::ID_3)]) << 24);
	}
	std::vector<uint8_t> getIDBytes() const override {
		return std::vector<uint8_t>{
		        data[static_cast<size_t>(FixedSizeIndex::ID_0)],
		        data[static_cast<size_t>(FixedSizeIndex::ID_1)],
		        data[static_cast<size_t>(FixedSizeIndex::ID_2)],
		        data[static_cast<size_t>(FixedSizeIndex::ID_3)]
		};
	}
	void setIDBytes(const std::vector<uint8_t>& id_bytes) override {
		if (id_bytes.size() != 4) {
			throw std::invalid_argument("ID must be exactly 4 bytes for fixed frame");
		}
		data[static_cast<size_t>(FixedSizeIndex::ID_0)] = id_bytes[0];
		data[static_cast<size_t>(FixedSizeIndex::ID_1)] = id_bytes[1];
		data[static_cast<size_t>(FixedSizeIndex::ID_2)] = id_bytes[2];
		data[static_cast<size_t>(FixedSizeIndex::ID_3)] = id_bytes[3];
	}
	bool isValidID() const override {
		// For fixed frames, any 4-byte ID is valid
		return true;
	}
	bool isValidLength() const override {
		uint8_t dlc = getDLC();
		return dlc <= 8;
	}
	std::vector<uint8_t> serialize() const override {
		std::vector<uint8_t> result;
		result.reserve(20);
		result.push_back(start_byte);
		result.push_back(msg_header);
		result.insert(result.end(), data.begin(), data.end());
		result.push_back(checksum);
		return result;
	}
	bool deserialize(const std::vector<uint8_t>& raw_data) override {
		if (raw_data.size() != 20) return false;

		// Validate start and header bytes
		if (raw_data[0] != start_byte || raw_data[1] != msg_header) return false;

		// Copy data and checksum
		std::copy(raw_data.begin() + 2, raw_data.begin() + 19, data.begin());
		checksum = raw_data[19];

		return isValidFrame();
	}
	FrameType getType() const override {
		uint8_t frame_type = getFrameType();
		if (frame_type == to_uint8(FrameType::STD_FIXED)) {
			return FrameType::STD_FIXED;
		} else if (frame_type == to_uint8(FrameType::EXT_FIXED)) {
			return FrameType::EXT_FIXED;
		} else {
			throw std::runtime_error("Unknown frame type in fixedSize");
		}
	}
	bool isValidFrame() const override {
		return isValidStart() && isValidLength() && isValidID();
	}
};
// Variable Length Frame structure
struct AdapterVariableFrame : public baseFrame
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
	uint8_t type; // Type byte
	std::vector<uint8_t> id;   // ID bytes (2 or 4 bytes, little-endian)
	std::vector<uint8_t> data; // Data bytes (0-8 bytes)
	const uint8_t end_byte = to_uint8(Constants::END_BYTE); // End byte (0x55)

	AdapterVariableFrame() : baseFrame(),
		end_byte(to_uint8(Constants::END_BYTE)) {
		// Initialize the first two bits (bits 7 and 6) of the type byte to 1
		type = to_uint8(Constants::VAR_BASE);
	}
	explicit AdapterVariableFrame(uint8_t frame_type, FrameFmtVar frame_fmt, uint8_t dlc) : baseFrame(),
		end_byte(to_uint8(Constants::END_BYTE)) {

		// Initialize the first two bits (bits 7 and 6) of the type byte to 1
		type = to_uint8(Constants::VAR_BASE);
		if (frame_type == to_uint8(FrameFmt::EXT)) {
			type |= 0x10;         // Set bit 5 for extended frame
		}
		if (frame_fmt == FrameFmtVar::REMOTE) {
			type |= 0x10;         // Set bit 4 for remote frame
		}
		type |= (dlc & 0x0F);         // Set bits 3-0 for DLC
	}

	// basic check to verify end byte
	bool isValidEnd() const
	{
		return end_byte == to_uint8(Constants::END_BYTE);
	}
	// basic check to verify frame length
	bool isValidLength() const
	{
		// Extract DLC from type byte (lower 4 bits)
		uint8_t dlc = type & 0x0F;
		// Check if data length matches DLC
		return data.size() == dlc;
	}

	// Safe data manipulation methods (kept for backward compatibility)
	void setDataBytes(const std::vector<uint8_t>& data_bytes)
	{
		setData(data_bytes);
	}
	void reserveCapacity(size_t id_size, size_t data_size)
	{
		id.reserve(id_size);
		data.reserve(data_size);
	}
	// STL interface and Operator overloading
	uint8_t* begin() override {
		return data.data();
	}
	uint8_t* end() override {
		return data.data() + data.size();
	}
	const uint8_t* begin() const override {
		return data.data();
	}
	const uint8_t* end() const override {
		return data.data() + data.size();
	}
	void fill(uint8_t value) override {
		std::fill(data.begin(), data.end(), value);
	}
	std::size_t size() const override {
		return 2 + id.size() + data.size() + 1; // start byte + type + id + data + end byte
	}
	void setData(const std::vector<uint8_t>& new_data) override {
		data = new_data;
		if (data.size() > 8) {
			data.resize(8); // CAN data max 8 bytes
		}
	}
	void setData(size_t index, uint8_t value) override {
		data.at(index) = value;
	}
	std::vector<uint8_t> getData() const override {
		return data;
	}
	uint8_t getDataAt(size_t index) const override {
		return data.at(index);
	}

	// Protocol Handling
	void setFrameType(uint8_t frame_type) override {
		// Update the frame type bit in the type byte
		type &= 0xDF; // Clear bit 5
		if (frame_type == to_uint8(FrameFmt::EXT)) {
			type |= 0x20; // Set bit 5 for extended frame
		}
	}
	uint8_t getFrameType() const override {
		return (type & 0x20) ? to_uint8(FrameFmt::EXT) : to_uint8(FrameFmt::STD);
	}
	void setDLC(uint8_t dlc) override {
		type = (type & 0xF0) | (dlc & 0x0F);
	}
	uint8_t getDLC() const override {
		return type & 0x0F;
	}
	void setID(uint32_t id_value) override {
		bool is_extended = (type & 0x20) != 0;
		if (is_extended) {
			// 4 bytes for extended ID (little-endian)
			id.resize(4);
			id[0] = static_cast<uint8_t>(id_value & 0xFF);
			id[1] = static_cast<uint8_t>((id_value >> 8) & 0xFF);
			id[2] = static_cast<uint8_t>((id_value >> 16) & 0xFF);
			id[3] = static_cast<uint8_t>((id_value >> 24) & 0xFF);
		} else {
			// 2 bytes for standard ID (little-endian)
			id.resize(2);
			id[0] = static_cast<uint8_t>(id_value & 0xFF);
			id[1] = static_cast<uint8_t>((id_value >> 8) & 0xFF);
		}
	}
	uint32_t getID() const override {
		uint32_t id_value = 0;
		if (id.size() == 4) {
			// Extended ID
			id_value = static_cast<uint32_t>(id[0]) |
			           (static_cast<uint32_t>(id[1]) << 8) |
			           (static_cast<uint32_t>(id[2]) << 16) |
			           (static_cast<uint32_t>(id[3]) << 24);
		} else if (id.size() == 2) {
			// Standard ID
			id_value = static_cast<uint32_t>(id[0]) |
			           (static_cast<uint32_t>(id[1]) << 8);
		}
		return id_value;
	}
	std::vector<uint8_t> getIDBytes() const override {
		return id;
	}
	void setIDBytes(const std::vector<uint8_t>& id_bytes) override {
		id = id_bytes;
		if (id.size() != 2 && id.size() != 4) {
			throw std::invalid_argument("ID size must be 2 or 4 bytes for AdapterVariableFrame");
		}
	}
	bool isValidID() const override {
		return id.size() == 2 || id.size() == 4;
	}
	std::vector<uint8_t> serialize() const override {
		std::vector<uint8_t> result;
		result.reserve(size());
		result.push_back(start_byte);
		result.push_back(type);
		result.insert(result.end(), id.begin(), id.end());
		result.insert(result.end(), data.begin(), data.end());
		result.push_back(end_byte);
		return result;
	}
	bool deserialize(const std::vector<uint8_t>& raw_data) override {
		if (raw_data.size() < 4) return false; // Minimum size: start + type + id(2) + end

		size_t idx = 0;
		// Check start byte
		if (raw_data[idx++] != start_byte) return false;

		// Get type byte
		type = raw_data[idx++];

		// Determine ID size based on frame type
		size_t id_size = ((type & 0x20) != 0) ? 4 : 2; // Extended or standard
		if (raw_data.size() < 2 + id_size + 1) return false; // start + type + id + end minimum

		// Extract ID
		id.assign(raw_data.begin() + idx, raw_data.begin() + idx + id_size);
		idx += id_size;

		// Extract DLC and validate
		uint8_t dlc = type & 0x0F;
		if (raw_data.size() != 2 + id_size + dlc + 1) return false; // Exact size check

		// Extract data
		data.assign(raw_data.begin() + idx, raw_data.begin() + idx + dlc);
		idx += dlc;

		// Check end byte
		return raw_data[idx] == end_byte;
	}
	USBCANFrameType getType() const override {
		bool is_extended = (type & 0x20) != 0;
		return is_extended ? FrameType::EXT_VAR : FrameType::STD_VAR;
	}
	bool isValidFrame() const override {
		return isValidStart() && isValidEnd() && isValidLength() && isValidID();
	}
	uint8_t& operator[](std::size_t index) override {
		if (index == 0) {
			return const_cast<uint8_t&>(start_byte);
		} else if (index == 1) {
			return type;
		} else if (index >= 2 && index < 2 + id.size()){
			return id[index - 2];
		} else if (index >= 2 + id.size() && index < 2 + id.size() + data.size()) {
			return data[index - 2 - id.size()];
		} else if (index == 2 + id.size() + data.size()) {
			return const_cast<uint8_t&>(end_byte);
		} else {
			throw std::out_of_range("Index out of range for AdapterVariableFrame");
		}
	};
	const uint8_t& operator[](std::size_t index) const override {
		if (index == 0) {
			return start_byte;
		} else if (index == 1) {
			return type;
		} else if (index >= 2 && index < 2 + id.size()){
			return id[index - 2];
		} else if (index >= 2 + id.size() && index < 2 + id.size() + data.size()) {
			return data[index - 2 - id.size()];
		} else if (index == 2 + id.size() + data.size()) {
			return end_byte;
		} else {
			throw std::out_of_range("Index out of range for AdapterVariableFrame");
		}
	}
	// at() method with bounds checking
	uint8_t& at(std::size_t index) override {
		return this->operator[](index);
	}
	const uint8_t& at(std::size_t index) const override {
		return this->operator[](index);
	}
};
#pragma pack(pop)



}
