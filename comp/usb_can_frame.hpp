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
	virtual void setData(FixedSizeIndex index, uint8_t value) = 0;
	virtual std::vector<uint8_t> getData() const = 0;
	virtual uint8_t getData(FixedSizeIndex index) const = 0;
	virtual const bool inDataRange(FixedSizeIndex index) const = 0;

	// Protocol Handling
	virtual void setType(Type type) = 0;
	virtual Type getType() const = 0;
	virtual void setFrameType(FrameType frame_type) = 0;
	virtual FrameType getFrameType() const = 0;
	virtual void setFrameFmt(FrameFmt frame_fmt) = 0;
	virtual FrameFmt getFrameFmt() const = 0;
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
	virtual bool isValidFrame() const = 0;
};

// Fixed Frame structure (20 bytes)
struct fixedSize : public baseFrame {
	/**
	 * @brief USB-CAN-A adapter 20-byte fixed frame structure
	 * This structure represents the 20-byte fixed frame format used by the USB-CAN-A and
	 * inherits from the base frame structure to include the start byte.
	 * @see usb_can_common.hpp for details on frame format
	 * @note This fixed frame format is less flexible than the variable length format
	 * @note The checksum is calculated as the low 8 bits of the cumulative sum from
	 * the type byte to the reserved byte (inclusive) and is stored in the last byte of the frame.
	 * The checksum is automatically updated before serialization but must be recalculated
	 * manually if the frame is modified directly.
	 */

	// Data bytes (17 bytes: type, frame type, ID[4], DLC, Data[8], reserved, checksum)
	const uint8_t msg_header = to_uint8(Constants::MSG_HEADER); // Message header byte
	uint8_t type;
	uint8_t frame_type;
	uint8_t frame_fmt;
	// ID bytes (4 bytes)
	std::array<uint8_t, 4> id_bytes;
	// DLC byte
	uint8_t dlc;
	// Data bytes (8 bytes)
	std::array<uint8_t, 8> data;
	// Reserved byte
	const uint8_t reserved = to_uint8(Constants::RESERVED0);
	// Checksum byte
	uint8_t checksum;

	// Constructor
	explicit fixedSize(Type type, FrameType frame_type,  FrameFmt fmt) : baseFrame(),
		type(to_uint8(type)),
		frame_type(to_uint8(frame_type)),
		frame_fmt(to_uint8(fmt)) {
		// Initialize ID, DLC, Data, and Checksum to zero
		id_bytes.fill(0);
		dlc = 0;
		data.fill(0);
		checksum = 0;
	}

	~fixedSize() = default;

	// Override operators
	uint8_t& operator[](std::size_t index) override {
		if (index == 0) {
			return const_cast<uint8_t&>(start_byte);
		} else if (index == 1) {
			return const_cast<uint8_t&>(msg_header);
		} else if (index == 2) {
			return type;
		} else if (index == 3) {
			return frame_type;
		} else if (index == 4) {
			return frame_fmt;
		} else if (index >= 5 && index < 9) {
			return id_bytes[index - 5];
		} else if (index == 9) {
			return dlc;
		} else if (index >= 10 && index < 17) {
			return data[index - 10];
		} else if (index == 18) {
			return const_cast<uint8_t&>(reserved);
		} else if (index == 19) {
			return checksum;
		} else {
			std::string msg = "Index " + std::to_string(index) + " is out of range for Waveshare fixedSize frame (0-19)";
			throw std::out_of_range(msg);
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
	const bool inDataRange(FixedSizeIndex index) const override {
		/**
		 * @brief Check if the index is in the valid data range
		 * This method checks if the provided index is within the valid range for
		 * accessing data bytes in the fixed size frame.
		 * Valid indices are from 10 to 17 (inclusive) corresponding to data[0] to data[7].
		 * @param index The index to check
		 * @return true if index is valid, false otherwise
		 */
		if (index < FixedSizeIndex::DATA_0 || index > FixedSizeIndex::DATA_7) {
			return false;
		}
		return true;
	}

	// STL interface
	uint8_t* begin() override {
		return &this->operator[](0);
	}
	uint8_t* end() override {
		return &this->operator[](20);
	}
	const uint8_t* begin() const override {
		return &this->operator[](0);
	}
	const uint8_t* end() const override {
		return &this->operator[](20);
	}
	void fill(uint8_t value) override {
		std::fill(this->begin(), this->end(), value);
	}
	std::size_t size() const override {
		return 20;
	}

	// Protocol Handling
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
	void setData(FixedSizeIndex index, uint8_t value) override {
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
		if (!inDataRange(index)) {
			std::string msg = "Index " + std::to_string(to_uint(index)) + " is out of range for Waveshare fixedSize data [10-17]";
			throw std::out_of_range(msg);
		}
		// Set the data byte at the specified index
		data.at(to_uint(index)-10) = value;
	}
	uint8_t getData(FixedSizeIndex index) const override {
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
		if (!inDataRange(index)) {
			std::string msg = "Index " + std::to_string(to_uint(index)) + " is out of range for Waveshare fixedSize data [10-17]";
			throw std::out_of_range(msg);
		}
		return data.at(to_uint(index)-10);
	}
	void setType(Type type) override {
		/**
		 * @brief Set the frame type byte in the data array
		 * @see usb_can_common.hpp for details on frame type
		 * @param type The frame type byte to set
		 * @throws std::invalid_argument if frame_type is not 0x01 or 0x02
		 */
		if (type != Type::DATA_FIXED &&
		    type != Type::CONF_FIXED) {
			std::string msg = "Invalid packet type: " +
			                  std::to_string(to_uint8(type)) +
			                  ". Must be " +
			                  std::to_string(to_uint8(Type::DATA_FIXED)) + " (data) or " +
			                  std::to_string(to_uint8(Type::CONF_FIXED)) + " (configuration)";
			throw std::invalid_argument(msg);
		}
		this->type = to_uint8(type);
	}
	void setFrameType(FrameType frame_type) override {
		/**
		 * @brief Set the frame type byte in the data array
		 * @see usb_can_common.hpp for details on frame type
		 * @param frame_type The frame type byte to set
		 * @throws std::invalid_argument if frame_type is not 0x01 or 0x02
		 */
		if (frame_type != FrameType::STD_FIXED &&
		    frame_type != FrameType::EXT_FIXED) {
			std::string msg = "Invalid frame type: " +
			                  std::to_string(to_uint8(frame_type)) +
			                  ". Must be " +
			                  std::to_string(to_uint8(FrameType::STD_FIXED)) + " (standard) or " +
			                  std::to_string(to_uint8(FrameType::EXT_FIXED)) + " (extended)";
			throw std::invalid_argument(msg);
		}
		this->frame_type = to_uint8(frame_type);
	}
	void setFrameFmt(FrameFmt frame_fmt) override {
		/**
		 * @brief Set the frame format byte in the data array
		 * @see usb_can_common.hpp for details on frame format
		 * @param frame_fmt The frame type byte to set
		 * @throws std::invalid_argument if frame_type is not 0x01 or 0x02
		 */
		if (frame_fmt != FrameFmt::DATA_FIXED &&
		    frame_fmt != FrameFmt::REMOTE_FIXED) {
			std::string msg = "Invalid frame format: " +
			                  std::to_string(to_uint8(frame_fmt)) +
			                  ". Must be " +
			                  std::to_string(to_uint8(FrameFmt::DATA_FIXED)) + " (data) or " +
			                  std::to_string(to_uint8(FrameFmt::REMOTE_FIXED)) + " (remote)";
			throw std::invalid_argument(msg);
		}
		this->frame_fmt = to_uint8(frame_fmt);
	}
	Type getType() const override {
		/**
		 * @brief Get the packet type byte.
		 * The packet type is stored in the {type} member variable and can be either
		 * 0x01 (data) or 0x02 (configuration).
		 * @return Packet type byte
		 */
		return static_cast<Type>(type);
	}
	FrameFmt getFrameFmt() const override {
		/**
		 * @brief Get the frame type from the data array
		 * Frame type is stored in the {frame_type} member variable and can be either
		 * 0x01 (standard) or 0x02 (extended).
		 * @see usb_can_common.hpp for details on frame format
		 * @return Frame type byte
		 */
		return static_cast<FrameFmt>(frame_fmt);
	}
	FrameType getFrameType() const override {
		/**
		 * @brief Get the frame format from the data array
		 * Frame format is stored in the {frame_fmt} member variable and can be either
		 * 0x01 (data frame) or 0x02 (remote frame).
		 * @see usb_can_common.hpp for details on frame format
		 * @return Frame format byte
		 */
		return static_cast<FrameType>(frame_type);
	}
	void setDLC(uint8_t dlc) override {
		/**
		 * @brief Set the Data Length Code (DLC) in the ID array
		 * The DLC is stored in {id_bytes} field and must be in the range 0 to 8 for CAN frames.
		 * @see usb_can_common.hpp for details on frame format
		 * @param dlc The DLC value to set (0-8)
		 * @throws std::invalid_argument if dlc is not in the range 0-8
		 */
		if (dlc > 8) {
			throw std::invalid_argument("DLC must be 0-8 for CAN frames");
		}
		this->dlc = dlc;
	}
	uint8_t getDLC() const override {
		/**
		 * @brief Get the Data Length Code (DLC) from the data array
		 * The DLC is stored in {id_bytes} field and must be in the range 0 to 8 for CAN frames.
		 * @see usb_can_common.hpp for details on frame format
		 * @return The DLC value (0-8)
		 */
		return dlc;
	}
	void setID(uint32_t id_value) override {
		/**
		 * @brief Set the CAN ID in the data array, converting an integer to bytes
		 * This method sets the CAN ID in the data array, converting a 32-bit unsigned
		 * integer to the appropriate little-endian byte representation.
		 * For standard frames, the lower (firsts) two bytes are used,
		 * and the upper (seconds) two bytes are zeroed out.
		 * @param id_value The CAN ID value to set as a 32-bit unsigned integer
		 * @throws std::invalid_argument if id_value is out of range
		 * @note The ID is stored in little-endian format
		 * @note Call this function only if you are sure about the ID being set
		 */
		const FrameType frame_type = getFrameType();

		// Validate ID range based on frame type
		if (frame_type == FrameType::STD_FIXED) {
			if (id_value > 0x7FF) {
				throw std::invalid_argument("Standard ID must be in range 0 to 2047 (0x7FF)");
			}
		} else if (frame_type == FrameType::EXT_FIXED) {
			if (id_value > 0x1FFFFFFF) {
				throw std::invalid_argument("Extended ID must be in range 0 to 536870911 (0x1FFFFFFF)");
			}
		}

		// Convert to little-endian byte representation
		id_bytes[0] = static_cast<uint8_t>(id_value & 0xFF);
		id_bytes[1] = static_cast<uint8_t>((id_value >> 8) & 0xFF);
		id_bytes[2] = static_cast<uint8_t>((id_value >> 16) & 0xFF);
		id_bytes[3] = static_cast<uint8_t>((id_value >> 24) & 0xFF);
	}
	uint32_t getID() const override {
		/**
		 * @brief Get the CAN ID from the data array, converting bytes to an integer
		 * This method retrieves the CAN ID from the data array, converting the
		 * little-endian byte representation to a 32-bit unsigned integer.
		 * For standard frames, only the lower (firsts) two bytes are used,
		 * and the upper (seconds) two bytes are ignored.
		 * @return The CAN ID value as a 32-bit unsigned integer
		 * @note The ID is stored in little-endian format
		 */
		return static_cast<uint32_t>(id_bytes[0]) |
		       (static_cast<uint32_t>(id_bytes[1]) << 8) |
		       (static_cast<uint32_t>(id_bytes[2]) << 16) |
		       (static_cast<uint32_t>(id_bytes[3]) << 24);
	}
	std::vector<uint8_t> getIDBytes() const override {
		/**
		 * @brief Get the CAN ID bytes from the data array
		 * This method retrieves the CAN ID bytes from the data array as a vector.
		 * It provides a safe way to access the internal ID byte array.
		 * @return A vector containing the 4 ID bytes
		 */
		return std::vector<uint8_t>{
		        data[static_cast<size_t>(FixedSizeIndex::ID_0)],
		        data[static_cast<size_t>(FixedSizeIndex::ID_1)],
		        data[static_cast<size_t>(FixedSizeIndex::ID_2)],
		        data[static_cast<size_t>(FixedSizeIndex::ID_3)]
		};
	}
	void setIDBytes(const std::vector<uint8_t>& id_bytes) override {
		/**
		 * @brief Set the CAN ID bytes in the data array
		 * This method sets the CAN ID bytes in the data array from a vector.
		 * It ensures that the ID size is exactly 4 bytes.
		 * @param id_bytes A vector containing the 4 ID bytes to set
		 * @throws std::invalid_argument if id_bytes size is not 4
		 * @note The ID is stored in little-endian format
		 * @note Call this function only if you are sure about the ID being set
		 */
		if (id_bytes.size() != 4) {
			throw std::invalid_argument("ID must be exactly 4 bytes for fixed frame");
		}

		// Set the ID bytes in little-endian format
		data[static_cast<size_t>(FixedSizeIndex::ID_0)] = id_bytes[0];
		data[static_cast<size_t>(FixedSizeIndex::ID_1)] = id_bytes[1];
		data[static_cast<size_t>(FixedSizeIndex::ID_2)] = id_bytes[2];
		data[static_cast<size_t>(FixedSizeIndex::ID_3)] = id_bytes[3];
	}
	void setChecksum(uint8_t checksum) {
		/**
		 * @brief Set the checksum byte in the data array
		 * This method sets the checksum byte in the data array.
		 * The checksum is stored in the {checksum} member variable.
		 * @param checksum The checksum byte to set
		 * @note The checksum is calculated over the first 19 bytes of the frame
		 */
		this->checksum = checksum;
	}
	uint8_t getChecksum() const {
		/**
		 * @brief Get the checksum byte from the data array
		 * This method retrieves the checksum byte from the data array.
		 * The checksum is stored in the {checksum} member variable.
		 * @return The checksum byte
		 * @note The checksum is calculated over the first 19 bytes of the frame
		 */
		return checksum;
	}

	//! Checksum calculation method
	uint8_t calculateChecksum() const {
		/**
		 * @brief Calculate the checksum for the fixed size frame
		 * This method calculates the checksum for the fixed size frame by summing
		 * all bytes from the {type} field to the {reserved} field and taking the lower 8 bits.
		 * The checksum is stored in the {checksum} member variable.
		 * @note The checksum is calculated over the first 19 bytes of the frame
		 */

		uint32_t sum = 0;
		// Sum all bytes from type to reserved
		sum += type;
		sum += frame_type;
		sum += frame_fmt;
		sum += std::accumulate(id_bytes.begin(), id_bytes.end(), 0);
		sum += dlc;
		sum += std::accumulate(data.begin(), data.end(), 0);
		//! This could be omitted as it's always 0x00
		sum += reserved;
		// Store the lower 8 bits of the sum as the checksum
		return static_cast<uint8_t>(sum & 0xFF);
	}

	// Validation methods
	bool isValidID() const override {
		/**
		 * @brief Validate the CAN ID based on frame type
		 * This method checks if the CAN ID is valid based on the frame type.
		 * For standard frames, the ID must be in the range 0 to 2047 (0x7FF).
		 * For extended frames, the ID must be in the range 0 to 536870911 (0x1FFFFFFF).
		 * @return true if the ID is valid, false otherwise
		 * @note This function does not check if the ID bytes are correctly set,
		 *       only if the ID value is within the valid range for the frame type
		 */
		const FrameType frame_type = getFrameType();
		const uint32_t id_value = getID();
		if (frame_type == FrameType::STD_FIXED) {
			return id_value <= 0x7FF;
		} else if (frame_type == FrameType::EXT_FIXED) {
			return id_value <= 0x1FFFFFFF;
		}
		return false; // Invalid frame type
	}
	bool isValidLength() const override {
		/**
		 * @brief Validate the Data Length Code (DLC)
		 * This method checks if the DLC is valid for CAN frames.
		 * The DLC must be in the range 0 to 8.
		 * @return true if the DLC is valid, false otherwise
		 * @note This function does not check if the data bytes are correctly set,
		 *       only if the DLC value is within the valid range for CAN frames
		 */
		uint8_t dlc = getDLC();
		return dlc <= 8;
	}
	bool isValidFrame() const override {
		/**
		 * @brief Validate the entire frame
		 * This method checks if the entire frame is valid by validating the start byte,
		 * length, ID, and checksum.
		 * @return true if the frame is valid, false otherwise
		 * @note This function combines all individual validation checks
		 */

		std::string errmsg;
		Type t = getType();
		FrameType ft = getFrameType();
		FrameFmt ff = getFrameFmt();
		uint8_t calc_checksum = calculateChecksum();

		// validate single components
		if (!isValidStart()) {
			errmsg = "Invalid start byte: " + std::to_string(start_byte) +
			         ", expected: " + std::to_string(to_uint8(Constants::START_BYTE));
			goto invalid;
		}
		if (!isValidLength()) {
			errmsg = "Invalid DLC: " + std::to_string(dlc) + ", must be 0-8";
			goto invalid;
		}
		if (!isValidID()) {
			errmsg = "Invalid ID: " + std::to_string(getID()) +
			         " for frame type: " + std::to_string(to_uint8(ft));
			goto invalid;
		}

		//* now be sure that the tuple type, frame_type, frame_fmt is valid
		//! Here we don't check if the frame is a configuration frame, as they will be handled separately

		//* if the type is data frame, then the format byte must be DATA_FIXED
		if (t == Type::DATA_FIXED) {
			if (ff != FrameFmt::DATA_FIXED) {
				errmsg = "Invalid frame format: " + std::to_string(to_uint8(ff)) +
				         " for data frame type: " + std::to_string(to_uint8(t));
				goto invalid;
			}
			//* format is correct, now check the frame type
			if (ft != FrameType::STD_FIXED && ft != FrameType::EXT_FIXED) {
				errmsg = "Invalid frame type: " + std::to_string(to_uint8(ft)) +
				         " for packet type: " + std::to_string(to_uint8(t));
				goto invalid;
			}
		}

		//* Now verify checksum
		if (calc_checksum != checksum)
			errmsg = "Invalid checksum: " + std::to_string(checksum) +
			         ", expected: " + std::to_string(calc_checksum);

invalid:
		if (!errmsg.empty()) {
			throw std::invalid_argument(errmsg);
		}
		return true;
	}

	// Serialization and Deserialization
	std::vector<uint8_t> serialize() const override {
		/**
		 * @brief Serialize the fixed size frame to a byte vector
		 * This method serializes the fixed size frame into a byte vector,
		 * including the start byte, message header, data bytes, and checksum.
		 * It ensures that the frame is valid before serialization.
		 * @return A vector containing the serialized frame bytes
		 * @throws std::invalid_argument if the frame is not valid
		 * @note The serialized frame is always 20 bytes long
		 * @note The checksum is calculated and included in the serialized frame at
		 * runtime to ensure integrity of the data being sent over USB without the need to
		 * lose time calling calculateChecksum() everytime that the frame is modified
		 */
		// Calculate checksum before serialization
		std::vector<uint8_t> result;
		result.reserve(20);
		result.push_back(start_byte);
		result.push_back(msg_header);
		result.insert(result.end(), data.begin(), data.end());
		result.push_back(checksum);
		// Ensure the frame is valid before serialization
		if (!isValidFrame()) {
			throw std::invalid_argument("Cannot serialize an invalid frame");
		}
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


};
// Variable Length Frame structure
struct varSize : public baseFrame
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

	varSize() : baseFrame(),
		end_byte(to_uint8(Constants::END_BYTE)) {
	}
	~varSize() = default;

	// Constructor with parameters
	explicit varSize(FrameType frame_type, FrameFmt frame_fmt, uint8_t dlc) : baseFrame(),
		end_byte(to_uint8(Constants::END_BYTE)) {

		this->type = make_variable_type_byte(frame_type, frame_fmt, dlc);
		// Initialize ID and Data vectors based on frame type and DLC
		if (frame_type == FrameType::STD_VAR) {
			id.resize(2, 0); // Standard ID (2 bytes)
		} else if (frame_type == FrameType::EXT_VAR) {
			id.resize(4, 0); // Extended ID (4 bytes)
		} else {
			throw std::invalid_argument("Invalid frame type for varSize constructor");
		}
		if (dlc > 8) {
			throw std::invalid_argument("DLC must be 0-8 for CAN frames");
		}
		data.resize(dlc, 0); // Data bytes (0-8 bytes)
	}

	//TODO Override operators
	uint8_t& operator[](std::size_t index) override {
		if (index == 0) {
			return const_cast<uint8_t&>(start_byte);
		} else if (index == 1) {
			return type;
		} else if (index >= 2 && index < 2 + id.size()) {
			return id[index - 2];
		} else if (index >= 2 + id.size() && index < 2 + id.size() + data.size()) {
			return data[index - 2 - id.size()];
		} else if (index == 2 + id.size() + data.size()) {
			return const_cast<uint8_t&>(end_byte);
		} else {
			std::string msg = "Index " + std::to_string(index) + " is out of range for Waveshare varSize frame (0-" +
			                  std::to_string(2 + id.size() + data.size()) + ")";
			throw std::out_of_range(msg);
		}
	}
	const uint8_t& operator[](std::size_t index) const override {
		return const_cast<varSize*>(this)->operator[](index);
	}
	uint8_t& at(std::size_t index) override {
		return this->operator[](index);
	}
	const uint8_t& at(std::size_t index) const override {
		return this->operator[](index);
	}


	//* Validation methods
	bool isValidEnd() const {
		/**
		 * @brief Check if the end byte is valid
		 * This method checks if the end byte is valid by comparing it to the expected value.
		 * The end byte is always 0x55 for variable frames.
		 * @return true if the end byte is valid, false otherwise
		 * @note This function is used in frame validation
		 */
		return end_byte == to_uint8(Constants::END_BYTE);
	}

	bool isValidLength() const override {
		/**
		 * @brief Validate the Data Length Code (DLC)
		 * This method checks if the DLC is valid for CAN frames.
		 * The DLC must be in the range 0 to 8 and must match the actual data length.
		 * @return true if the DLC is valid, false otherwise
		 * @note This function does not check if the data bytes are correctly set,
		 *       only if the DLC value is within the valid range for CAN frames
		 */
		// Extract DLC from type byte (lower 4 bits)
		uint8_t dlc = type & 0x0F;
		// Check if data length matches DLC
		return data.size() == dlc;
	}

	bool isValidFrame() const override {
		/**
		 * @brief Validate the entire variable size frame
		 * This method checks if the entire frame is valid by validating the start byte,
		 * end byte, length, and ID.
		 * !Variable Frames cannot be configuration frames, so we don't check for that
		 * @return true if the frame is valid, false otherwise
		 * @note This function combines all individual validation checks
		 */

		std::string errmsg;
		Type t = getType();
		FrameType ft = getFrameType();
		FrameFmt ff = getFrameFmt();

		// validate single components
		if (!isValidStart()) {
			errmsg = "Invalid start byte: " + std::to_string(start_byte) +
			         ", expected: " + std::to_string(to_uint8(Constants::START_BYTE));
			goto invalid;
		}
		if (!isValidEnd()) {
			errmsg = "Invalid end byte: " + std::to_string(end_byte) +
			         ", expected: " + std::to_string(to_uint8(Constants::END_BYTE));
			goto invalid;
		}
		if (!isValidLength()) {
			errmsg = "Invalid DLC: " + std::to_string(getDLC()) +
			         ", data length is: " + std::to_string(data.size());
			goto invalid;
		}
		if (!isValidID()) {
			errmsg = "Invalid ID: " + std::to_string(getID()) +
			         " for frame type: " + std::to_string(to_uint8(ft));
			goto invalid;
		}

		//* now be sure that the tuple type, frame_type, frame_fmt is valid

		//* if the type is data frame, then the format byte must be DATA_VAR
		if (t == Type::DATA_VAR) {
			if (ff != FrameFmt::DATA_VAR) {
				errmsg = "Invalid frame format: " + std::to_string(to_uint8(ff)) +
				         " for data frame type: " + std::to_string(to_uint8(t));
				goto invalid;
			}
			//* format is correct, now check the frame type
			if (ft != FrameType::STD_VAR && ft != FrameType::EXT_VAR) {
				errmsg = "Invalid frame type: " + std::to_string(to_uint8(ft)) +
				         " for packet type: " + std::to_string(to_uint8(t));
				goto invalid;
			}
		}
invalid:
		if (!errmsg.empty()) {
			throw std::invalid_argument(errmsg);
		}
		return true;
	}

};
#pragma pack(pop)



}
