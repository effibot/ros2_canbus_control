/**
 * @file usb_can_frame_class_version.hpp
 * @brief USB-CAN adapter frame handling for ROS2 - Class-based version
 * This file defines classes (instead of structs) to handle USB-CAN-A adapter frames,
 * implementing the Waveshare USB-CAN-A adapter protocol.
 * The implementation supports both fixed 20-byte frames and variable length frames, with
 * standard and extended CAN IDs.
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
 * ├── * Adapter20ByteFrame (fixed 20-byte frame)
 * │   ├── STD20ByteFrame (standard ID frame)
 * │   ├── EXT20ByteFrame (extended ID frame)
 * │   └── SettingsFrame (settings frame)
 * └── * AdapterVariableFrame (variable length frame)
 *     ├── STDVariableFrame (standard ID frame)
 *     └── EXTVariableFrame (extended ID frame)
 *
 * @author Andrea Efficace
 * @date September 2025
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

/**
 * @brief Abstract base class for all USB-CAN adapter frames
 * This class provides the common interface and basic functionality for all frame types.
 * It encapsulates the start byte and defines pure virtual methods that derived classes
 * must implement for specific frame handling.
 */
class AdapterBaseFrame {
protected:
const uint8_t start_byte_;

public:
// Constructor
explicit AdapterBaseFrame() : start_byte_(to_uint8(Constants::START_BYTE)) {
}

// Virtual destructor for proper cleanup
virtual ~AdapterBaseFrame() = default;

// Disable copy constructor and copy assignment operator to prevent slicing
AdapterBaseFrame(const AdapterBaseFrame&) = delete;
AdapterBaseFrame& operator=(const AdapterBaseFrame&) = delete;

// Enable move semantics
AdapterBaseFrame(AdapterBaseFrame&&) = default;
AdapterBaseFrame& operator=(AdapterBaseFrame&&) = default;

// Basic validation
bool isValidStart() const {
	return start_byte_ == to_uint8(Constants::START_BYTE);
}

// Pure virtual interface - must be implemented by derived classes
virtual uint8_t& operator[](std::size_t index) = 0;
virtual const uint8_t& operator[](std::size_t index) const = 0;
virtual uint8_t& at(std::size_t index) = 0;
virtual const uint8_t& at(std::size_t index) const = 0;

// STL-style interface
virtual uint8_t* begin() = 0;
virtual uint8_t* end() = 0;
virtual const uint8_t* begin() const = 0;
virtual const uint8_t* end() const = 0;
virtual void fill(uint8_t value) = 0;
virtual std::size_t size() const = 0;

// Data manipulation interface
virtual void setData(const std::vector<uint8_t>& new_data) = 0;
virtual void setData(size_t index, uint8_t value) = 0;
virtual std::vector<uint8_t> getData() const = 0;
virtual uint8_t getData(size_t index) const = 0;
virtual bool inDataRange(size_t index) const = 0;

// Protocol handling interface
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

// Serialization interface
virtual std::vector<uint8_t> serialize() const = 0;
virtual bool deserialize(const std::vector<uint8_t>& data) = 0;
virtual bool isValidFrame() const = 0;
};

/**
 * @brief Fixed 20-byte frame class
 * This class represents the 20-byte fixed frame format used by the USB-CAN-A adapter.
 * It provides encapsulation of frame data and safe access methods.
 */
class FixedSizeFrame : public AdapterBaseFrame {
private:
const uint8_t msg_header_;
uint8_t type_;
uint8_t frame_type_;
uint8_t frame_fmt_;
std::array<uint8_t, 4> id_bytes_;
uint8_t dlc_;
std::array<uint8_t, 8> data_;
const uint8_t reserved_;
uint8_t checksum_;

// Private helper methods
uint8_t calculateChecksum() const {
	uint32_t sum = 0;
	sum += type_;
	sum += frame_type_;
	sum += frame_fmt_;
	sum += std::accumulate(id_bytes_.begin(), id_bytes_.end(), 0);
	sum += dlc_;
	sum += std::accumulate(data_.begin(), data_.end(), 0);
	sum += reserved_;
	return static_cast<uint8_t>(sum & 0xFF);
}

void validateIndex(std::size_t index) const {
	if (index >= 20) {
		throw std::out_of_range("Index " + std::to_string(index) +
		                        " out of range for FixedSizeFrame (0-19)");
	}
}

public:
// Constructor
explicit FixedSizeFrame(Type type, FrameType frame_type, FrameFmt fmt)
	: AdapterBaseFrame(),
	msg_header_(to_uint8(Constants::MSG_HEADER)),
	type_(to_uint8(type)),
	frame_type_(to_uint8(frame_type)),
	frame_fmt_(to_uint8(fmt)),
	reserved_(to_uint8(Constants::RESERVED0)),
	checksum_(0) {

	// Validate input parameters
	if (type != Type::DATA_FIXED && type != Type::CONF_FIXED) {
		throw std::invalid_argument("Invalid type for FixedSizeFrame");
	}
	if (frame_type != FrameType::STD_FIXED && frame_type != FrameType::EXT_FIXED) {
		throw std::invalid_argument("Invalid frame type for FixedSizeFrame");
	}
	if (fmt != FrameFmt::DATA_FIXED && fmt != FrameFmt::REMOTE_FIXED) {
		throw std::invalid_argument("Invalid frame format for FixedSizeFrame");
	}

	// Initialize arrays
	id_bytes_.fill(0);
	dlc_ = 0;
	data_.fill(0);
	checksum_ = calculateChecksum();
}

// Destructor
~FixedSizeFrame() override = default;

// Index access operators
uint8_t& operator[](std::size_t index) override {
	validateIndex(index);
	switch (index) {
	case to_uint(FixedSizeIndex::START):
		return const_cast<uint8_t&>(start_byte_);
	case to_uint(FixedSizeIndex::HEADER):
		return const_cast<uint8_t&>(msg_header_);
	case to_uint(FixedSizeIndex::TYPE):
		return type_;
	case to_uint(FixedSizeIndex::FRAME_TYPE):
		return frame_type_;
	case to_uint(FixedSizeIndex::FRAME_FMT):
		return frame_fmt_;
	case to_uint(FixedSizeIndex::ID_0):
		return id_bytes_[0];
	case to_uint(FixedSizeIndex::ID_1):
		return id_bytes_[1];
	case to_uint(FixedSizeIndex::ID_2):
		return id_bytes_[2];
	case to_uint(FixedSizeIndex::ID_3):
		return id_bytes_[3];
	case to_uint(FixedSizeIndex::DLC):
		return dlc_;
	case to_uint(FixedSizeIndex::DATA_0):
		return data_[0];
	case to_uint(FixedSizeIndex::DATA_1):
		return data_[1];
	case to_uint(FixedSizeIndex::DATA_2):
		return data_[2];
	case to_uint(FixedSizeIndex::DATA_3):
		return data_[3];
	case to_uint(FixedSizeIndex::DATA_4):
		return data_[4];
	case to_uint(FixedSizeIndex::DATA_5):
		return data_[5];
	case to_uint(FixedSizeIndex::DATA_6):
		return data_[6];
	case to_uint(FixedSizeIndex::DATA_7):
		return data_[7];
	case to_uint(FixedSizeIndex::RESERVED):
		return const_cast<uint8_t&>(reserved_);
	case to_uint(FixedSizeIndex::CHECKSUM):
		return checksum_;
	default:
		throw std::out_of_range("Unhandled index for FixedSizeFrame");
	}
}

const uint8_t& operator[](std::size_t index) const override {
	return const_cast<FixedSizeFrame*>(this)->operator[](index);
}

uint8_t& at(std::size_t index) override {
	return this->operator[](index);
}

const uint8_t& at(std::size_t index) const override {
	return this->operator[](index);
}

// STL-style interface
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

// Data range validation
bool inDataRange(size_t index) const override {
	return index <= 7;
}

// Data manipulation methods
void setData(const std::vector<uint8_t>& new_data) override {
	if (new_data.size() > data_.size()) {
		throw std::invalid_argument("Data size " + std::to_string(new_data.size()) +
		                            " exceeds maximum of " + std::to_string(data_.size()));
	}

	std::size_t bytes_to_copy = std::min(new_data.size(), data_.size());
	std::copy(new_data.begin(), new_data.begin() + bytes_to_copy, data_.begin());

	if (bytes_to_copy < data_.size()) {
		std::fill(data_.begin() + bytes_to_copy, data_.end(), 0);
	}

	checksum_ = calculateChecksum();
}

void setData(size_t index, uint8_t value) override {
	if (!inDataRange(index)) {
		throw std::out_of_range("Data index " + std::to_string(index) + " out of range");
	}
	data_.at(index) = value;
	checksum_ = calculateChecksum();
}

std::vector<uint8_t> getData() const override {
	return std::vector<uint8_t>(data_.begin(), data_.end());
}

uint8_t getData(size_t index) const override {
	if (!inDataRange(index)) {
		throw std::out_of_range("Data index " + std::to_string(index) + " out of range");
	}
	return data_.at(index);
}

// Protocol handling methods
void setType(Type type) override {
	if (type != Type::DATA_FIXED && type != Type::CONF_FIXED) {
		throw std::invalid_argument("Invalid type for FixedSizeFrame");
	}
	type_ = to_uint8(type);
	checksum_ = calculateChecksum();
}

Type getType() const override {
	return static_cast<Type>(type_);
}

void setFrameType(FrameType frame_type) override {
	if (frame_type != FrameType::STD_FIXED && frame_type != FrameType::EXT_FIXED) {
		throw std::invalid_argument("Invalid frame type for FixedSizeFrame");
	}
	frame_type_ = to_uint8(frame_type);
	checksum_ = calculateChecksum();
}

FrameType getFrameType() const override {
	return static_cast<FrameType>(frame_type_);
}

void setFrameFmt(FrameFmt frame_fmt) override {
	if (frame_fmt != FrameFmt::DATA_FIXED && frame_fmt != FrameFmt::REMOTE_FIXED) {
		throw std::invalid_argument("Invalid frame format for FixedSizeFrame");
	}
	frame_fmt_ = to_uint8(frame_fmt);
	checksum_ = calculateChecksum();
}

FrameFmt getFrameFmt() const override {
	return static_cast<FrameFmt>(frame_fmt_);
}

void setDLC(uint8_t dlc) override {
	if (dlc > 8) {
		throw std::invalid_argument("DLC must be 0-8 for CAN frames");
	}
	dlc_ = dlc;
	checksum_ = calculateChecksum();
}

uint8_t getDLC() const override {
	return dlc_;
}

bool isValidLength() const override {
	return dlc_ <= 8;
}

void setID(uint32_t id_value) override {
	const FrameType frame_type = getFrameType();

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
	id_bytes_[0] = static_cast<uint8_t>(id_value & 0xFF);
	id_bytes_[1] = static_cast<uint8_t>((id_value >> 8) & 0xFF);
	id_bytes_[2] = static_cast<uint8_t>((id_value >> 16) & 0xFF);
	id_bytes_[3] = static_cast<uint8_t>((id_value >> 24) & 0xFF);

	checksum_ = calculateChecksum();
}

uint32_t getID() const override {
	return static_cast<uint32_t>(id_bytes_[0]) |
	       (static_cast<uint32_t>(id_bytes_[1]) << 8) |
	       (static_cast<uint32_t>(id_bytes_[2]) << 16) |
	       (static_cast<uint32_t>(id_bytes_[3]) << 24);
}

std::vector<uint8_t> getIDBytes() const override {
	return std::vector<uint8_t>(id_bytes_.begin(), id_bytes_.end());
}

void setIDBytes(const std::vector<uint8_t>& id_bytes) override {
	if (id_bytes.size() != 4) {
		throw std::invalid_argument("ID must be exactly 4 bytes for fixed frame");
	}
	std::copy(id_bytes.begin(), id_bytes.end(), id_bytes_.begin());
	checksum_ = calculateChecksum();
}

bool isValidID() const override {
	const FrameType frame_type = getFrameType();
	const uint32_t id_value = getID();

	if (frame_type == FrameType::STD_FIXED) {
		return id_value <= 0x7FF;
	} else if (frame_type == FrameType::EXT_FIXED) {
		return id_value <= 0x1FFFFFFF;
	}
	return false;
}

// Checksum methods
uint8_t getChecksum() const {
	return checksum_;
}

void updateChecksum() {
	checksum_ = calculateChecksum();
}

// Validation and serialization
bool isValidFrame() const override {
	if (!isValidStart()) return false;
	if (!isValidLength()) return false;
	if (!isValidID()) return false;

	uint8_t calc_checksum = calculateChecksum();
	if (calc_checksum != checksum_) return false;

	return true;
}

std::vector<uint8_t> serialize() const override {
	if (!isValidFrame()) {
		throw std::invalid_argument("Cannot serialize an invalid frame");
	}

	std::vector<uint8_t> result;
	result.reserve(20);

	result.push_back(start_byte_);
	result.push_back(msg_header_);
	result.push_back(type_);
	result.push_back(frame_type_);
	result.push_back(frame_fmt_);
	result.insert(result.end(), id_bytes_.begin(), id_bytes_.end());
	result.push_back(dlc_);
	result.insert(result.end(), data_.begin(), data_.end());
	result.push_back(reserved_);
	result.push_back(checksum_);

	return result;
}

bool deserialize(const std::vector<uint8_t>& raw_data) override {
	if (raw_data.size() != 20) return false;

	if (raw_data[0] != start_byte_ || raw_data[1] != msg_header_) return false;

	type_ = raw_data[2];
	frame_type_ = raw_data[3];
	frame_fmt_ = raw_data[4];
	std::copy(raw_data.begin() + 5, raw_data.begin() + 9, id_bytes_.begin());
	dlc_ = raw_data[9];
	std::copy(raw_data.begin() + 10, raw_data.begin() + 18, data_.begin());
	checksum_ = raw_data[19];

	return isValidFrame();
}
};

/**
 * @brief Variable length frame class
 * This class represents the variable length frame format used by the USB-CAN-A adapter.
 * It provides dynamic sizing based on the frame type and data length.
 */
class VariableSizeFrame : public AdapterBaseFrame {
private:
const uint8_t type_header_;
uint8_t frame_type_;
uint8_t frame_fmt_;
uint8_t dlc_;
std::vector<uint8_t> id_;
std::vector<uint8_t> data_;
const uint8_t end_byte_;

// Utility indices for frame structure
VarSizeIndex ID_END_;
VarSizeIndex DATA_START_;
VarSizeIndex DATA_END_;
VarSizeIndex FRAME_END_;

void updateIndices() {
	ID_END_ = VarSizeIndex::ID_START + id_.size() - 3;
	DATA_START_ = ID_END_ + 1;
	DATA_END_ = DATA_START_ + data_.size() - 1;
	FRAME_END_ = DATA_END_ + 1;
}

void validateIndex(std::size_t index) const {
	if (index > to_uint(FRAME_END_)) {
		throw std::out_of_range("Index " + std::to_string(index) +
		                        " out of range for VariableSizeFrame");
	}
}

public:
// Constructor
explicit VariableSizeFrame(FrameType frame_type, FrameFmt frame_fmt, uint8_t dlc)
	: AdapterBaseFrame(),
	type_header_(to_uint8(Type::DATA_VAR)),
	end_byte_(to_uint8(Constants::END_BYTE)) {

	// Validate and set frame type
	if (frame_type != FrameType::STD_VAR && frame_type != FrameType::EXT_VAR) {
		throw std::invalid_argument("Invalid frame type for VariableSizeFrame");
	}
	frame_type_ = to_uint8(frame_type);

	// Validate and set frame format
	if (frame_fmt != FrameFmt::DATA_VAR && frame_fmt != FrameFmt::REMOTE_VAR) {
		throw std::invalid_argument("Invalid frame format for VariableSizeFrame");
	}
	frame_fmt_ = to_uint8(frame_fmt);

	// Validate and set DLC
	if (dlc > 8) {
		throw std::invalid_argument("DLC must be 0-8 for CAN frames");
	}
	dlc_ = dlc;

	// Initialize ID vector based on frame type
	if (frame_type == FrameType::STD_VAR) {
		id_.resize(2, 0);
	} else {
		id_.resize(4, 0);
	}

	// Initialize data vector based on DLC
	data_.resize(dlc, 0);

	updateIndices();
}

// Default constructor (discouraged - use parameterized constructor)
VariableSizeFrame()
	: AdapterBaseFrame(),
	type_header_(to_uint8(Type::DATA_VAR)),
	frame_type_(0),
	frame_fmt_(0),
	dlc_(0),
	end_byte_(to_uint8(Constants::END_BYTE)) {
	id_.resize(2, 0);
	data_.resize(0, 0);
	updateIndices();
}

// Destructor
~VariableSizeFrame() override = default;

// Index access operators
uint8_t& operator[](std::size_t index) override {
	validateIndex(index);

	switch (index) {
	case 0:
		return const_cast<uint8_t&>(start_byte_);
	case 1:
		return const_cast<uint8_t&>(type_header_);
	case 2:
		return frame_type_;
	case 3:
		return frame_fmt_;
	case 4:
		return dlc_;
	default:
		if (index < to_uint(DATA_START_)) {
			return id_[index - 2];
		}
		if (index < to_uint(FRAME_END_)) {
			return data_[index - to_uint(DATA_START_)];
		}
		return const_cast<uint8_t&>(end_byte_);
	}
}

const uint8_t& operator[](std::size_t index) const override {
	return const_cast<VariableSizeFrame*>(this)->operator[](index);
}

uint8_t& at(std::size_t index) override {
	return this->operator[](index);
}

const uint8_t& at(std::size_t index) const override {
	return this->operator[](index);
}

// STL-style interface
uint8_t* begin() override {
	return &this->operator[](0);
}

uint8_t* end() override {
	return &this->operator[](to_uint(FRAME_END_) + 1);
}

const uint8_t* begin() const override {
	return &this->operator[](0);
}

const uint8_t* end() const override {
	return &this->operator[](to_uint(FRAME_END_) + 1);
}

void fill(uint8_t value) override {
	std::fill(this->begin(), this->end(), value);
}

std::size_t size() const override {
	size_t constants_size = 3; // start byte + type byte + end byte
	return constants_size + id_.size() + data_.size();
}

// Data range validation
bool inDataRange(size_t index) const override {
	return index < data_.size();
}

// Data manipulation methods
void setData(const std::vector<uint8_t>& new_data) override {
	if (new_data.size() > 8) {
		throw std::invalid_argument("Data size " + std::to_string(new_data.size()) +
		                            " exceeds maximum of 8");
	}

	data_ = new_data;
	dlc_ = static_cast<uint8_t>(data_.size());
	updateIndices();
}

void setData(size_t index, uint8_t value) override {
	if (!inDataRange(index)) {
		throw std::out_of_range("Data index " + std::to_string(index) + " out of range");
	}
	data_.at(index) = value;
}

std::vector<uint8_t> getData() const override {
	return data_;
}

uint8_t getData(size_t index) const override {
	if (!inDataRange(index)) {
		throw std::out_of_range("Data index " + std::to_string(index) + " out of range");
	}
	return data_.at(index);
}

// Protocol handling methods
void setType(Type type) override {
	if (type != Type::DATA_VAR) {
		throw std::invalid_argument("Invalid type for VariableSizeFrame - must be DATA_VAR");
	}
	// Type is always DATA_VAR for variable frames
}

Type getType() const override {
	return static_cast<Type>(type_header_);
}

void setFrameType(FrameType frame_type) override {
	if (frame_type != FrameType::STD_VAR && frame_type != FrameType::EXT_VAR) {
		throw std::invalid_argument("Invalid frame type for VariableSizeFrame");
	}

	frame_type_ = to_uint8(frame_type);

	// Resize ID vector based on frame type
	if (frame_type == FrameType::STD_VAR) {
		id_.resize(2, 0);
	} else {
		id_.resize(4, 0);
	}

	updateIndices();
}

FrameType getFrameType() const override {
	return static_cast<FrameType>(frame_type_);
}

void setFrameFmt(FrameFmt frame_fmt) override {
	if (frame_fmt != FrameFmt::DATA_VAR && frame_fmt != FrameFmt::REMOTE_VAR) {
		throw std::invalid_argument("Invalid frame format for VariableSizeFrame");
	}
	frame_fmt_ = to_uint8(frame_fmt);
}

FrameFmt getFrameFmt() const override {
	return static_cast<FrameFmt>(frame_fmt_);
}

void setDLC(uint8_t dlc) override {
	if (dlc > 8) {
		throw std::invalid_argument("DLC must be 0-8 for CAN frames");
	}
	dlc_ = dlc;
	// Note: This doesn't resize data vector to prevent data loss
	// Use setData() to change actual data size
}

uint8_t getDLC() const override {
	return dlc_;
}

bool isValidLength() const override {
	return dlc_ <= 8 && dlc_ == data_.size();
}

void setID(uint32_t id_value) override {
	const FrameType frame_type = getFrameType();

	if (frame_type == FrameType::STD_VAR) {
		if (id_value > 0x7FF) {
			throw std::invalid_argument("Standard ID must be in range 0 to 2047 (0x7FF)");
		}
		if (id_.size() != 2) id_.resize(2, 0);
		id_[0] = static_cast<uint8_t>(id_value & 0xFF);
		id_[1] = static_cast<uint8_t>((id_value >> 8) & 0xFF);
	} else if (frame_type == FrameType::EXT_VAR) {
		if (id_value > 0x1FFFFFFF) {
			throw std::invalid_argument("Extended ID must be in range 0 to 536870911 (0x1FFFFFFF)");
		}
		if (id_.size() != 4) id_.resize(4, 0);
		id_[0] = static_cast<uint8_t>(id_value & 0xFF);
		id_[1] = static_cast<uint8_t>((id_value >> 8) & 0xFF);
		id_[2] = static_cast<uint8_t>((id_value >> 16) & 0xFF);
		id_[3] = static_cast<uint8_t>((id_value >> 24) & 0xFF);
	} else {
		throw std::invalid_argument("Invalid frame type for setting ID");
	}

	updateIndices();
}

uint32_t getID() const override {
	uint32_t id_value = 0;
	if (id_.size() >= 2) {
		id_value |= static_cast<uint32_t>(id_[0]);
		id_value |= static_cast<uint32_t>(id_[1]) << 8;
	}
	if (id_.size() == 4) {
		id_value |= static_cast<uint32_t>(id_[2]) << 16;
		id_value |= static_cast<uint32_t>(id_[3]) << 24;
	}
	return id_value;
}

std::vector<uint8_t> getIDBytes() const override {
	return id_;
}

void setIDBytes(const std::vector<uint8_t>& id_bytes) override {
	const FrameType frame_type = getFrameType();

	if (frame_type == FrameType::STD_VAR && id_bytes.size() != 2) {
		throw std::invalid_argument("ID must be exactly 2 bytes for standard frame");
	}
	if (frame_type == FrameType::EXT_VAR && id_bytes.size() != 4) {
		throw std::invalid_argument("ID must be exactly 4 bytes for extended frame");
	}

	id_ = id_bytes;
	updateIndices();
}

bool isValidID() const override {
	const FrameType frame_type = getFrameType();
	const uint32_t id_value = getID();

	if (frame_type == FrameType::STD_VAR) {
		return id_value <= 0x7FF && id_.size() == 2;
	} else if (frame_type == FrameType::EXT_VAR) {
		return id_value <= 0x1FFFFFFF && id_.size() == 4;
	}
	return false;
}

// Validation and serialization
bool isValidFrame() const override {
	if (!isValidStart()) return false;
	if (type_header_ != to_uint8(Type::DATA_VAR)) return false;
	if (!isValidLength()) return false;
	if (!isValidID()) return false;

	// Validate frame type and format
	if (frame_type_ != to_uint8(FrameType::STD_VAR) &&
	    frame_type_ != to_uint8(FrameType::EXT_VAR)) return false;
	if (frame_fmt_ != to_uint8(FrameFmt::DATA_VAR) &&
	    frame_fmt_ != to_uint8(FrameFmt::REMOTE_VAR)) return false;

	return true;
}

std::vector<uint8_t> serialize() const override {
	if (!isValidFrame()) {
		throw std::invalid_argument("Cannot serialize an invalid frame");
	}

	std::vector<uint8_t> result;
	result.reserve(size());

	result.push_back(start_byte_);

	// Construct type byte from individual components
	uint8_t type_byte = make_variable_type_byte(getFrameType(), getFrameFmt(), getDLC());
	result.push_back(type_byte);

	result.insert(result.end(), id_.begin(), id_.end());
	result.insert(result.end(), data_.begin(), data_.end());
	result.push_back(end_byte_);

	return result;
}

bool deserialize(const std::vector<uint8_t>& raw_data) override {
	if (raw_data.size() < 6 || raw_data.size() > 15) return false;

	if (raw_data[0] != to_uint8(Constants::START_BYTE) ||
	    raw_data[raw_data.size() - 1] != to_uint8(Constants::END_BYTE)) return false;

	// Extract type byte and parse components
	uint8_t type_byte = raw_data[1];
	FrameType ft;
	FrameFmt ff;
	uint8_t dlc;
	parse_variable_type_byte(type_byte, ft, ff, dlc);

	try {
		setFrameType(ft);
		setFrameFmt(ff);
		setDLC(dlc);
	} catch (const std::invalid_argument&) {
		return false;
	}

	// Calculate expected sizes
	size_t expected_id_size = (ft == FrameType::STD_VAR) ? 2 : 4;
	size_t expected_data_size = dlc;
	size_t expected_total_size = 1 + 1 + expected_id_size + expected_data_size + 1;

	if (raw_data.size() != expected_total_size) return false;

	// Resize vectors and update indices
	id_.resize(expected_id_size);
	data_.resize(expected_data_size);
	updateIndices();

	// Copy ID and data bytes
	std::copy(raw_data.begin() + 2, raw_data.begin() + 2 + expected_id_size, id_.begin());
	std::copy(raw_data.begin() + 2 + expected_id_size,
	          raw_data.begin() + 2 + expected_id_size + expected_data_size, data_.begin());

	return isValidFrame();
}
};

#pragma pack(pop)

/**
 * @brief Factory class for creating frame objects
 * This class provides static methods to create frame objects using the builder pattern.
 * It ensures proper initialization and validation of frame parameters.
 */
class FrameFactory {
public:
// Create fixed size frame
static std::unique_ptr<FixedSizeFrame> createFixedFrame(
	Type type, FrameType frame_type, FrameFmt fmt) {
	return std::make_unique<FixedSizeFrame>(type, frame_type, fmt);
}

// Create variable size frame
static std::unique_ptr<VariableSizeFrame> createVariableFrame(
	FrameType frame_type, FrameFmt frame_fmt, uint8_t dlc = 0) {
	return std::make_unique<VariableSizeFrame>(frame_type, frame_fmt, dlc);
}

// Create frame from raw data (auto-detect type)
static std::unique_ptr<AdapterBaseFrame> createFrameFromData(
	const std::vector<uint8_t>& raw_data) {

	if (raw_data.empty()) {
		throw std::invalid_argument("Empty data provided");
	}

	if (raw_data[0] != to_uint8(Constants::START_BYTE)) {
		throw std::invalid_argument("Invalid start byte");
	}

	// Determine frame type based on size and structure
	if (raw_data.size() == 20 && raw_data[1] == to_uint8(Constants::MSG_HEADER)) {
		// Fixed frame
		auto frame = std::make_unique<FixedSizeFrame>(
			Type::DATA_FIXED, FrameType::STD_FIXED, FrameFmt::DATA_FIXED);
		if (frame->deserialize(raw_data)) {
			return frame;
		}
	} else if (raw_data.size() >= 6 && raw_data.size() <= 15 &&
	           raw_data[raw_data.size() - 1] == to_uint8(Constants::END_BYTE)) {
		// Variable frame
		auto frame = std::make_unique<VariableSizeFrame>(
			FrameType::STD_VAR, FrameFmt::DATA_VAR, 0);
		if (frame->deserialize(raw_data)) {
			return frame;
		}
	}

	throw std::invalid_argument("Unable to parse frame data");
}
};

} // namespace USBCANBridge
