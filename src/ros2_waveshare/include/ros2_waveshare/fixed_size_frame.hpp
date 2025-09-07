/**
 * @file fixed_size_frame.hpp
 * @brief Fixed 20-byte frame class for USB-CAN adapter
 * This file defines the FixedSizeFrame class that represents the 20-byte fixed frame
 * format used by the USB-CAN-A adapter. It provides encapsulation of frame data
 * and safe access methods.
 *
 * @author Andrea Efficace
 * @date September 2025
 */

#pragma once

#include "adapter_base_frame.hpp"
#include "usb_can_common.hpp"

#include <array>
#include <numeric>
#include <algorithm>
#include <stdexcept>
#include <string>

namespace USBCANBridge
{
#pragma pack(push, 1)

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

#pragma pack(pop)

} // namespace USBCANBridge
