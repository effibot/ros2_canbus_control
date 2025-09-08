/**
 * @file fixed_size_frame.cpp
 * @brief Implementation of Fixed 20-byte frame class for USB-CAN adapter
 *
 * @author Andrea Efficace
 * @date September 2025
 */

#include "ros2_waveshare/fixed_size_frame.hpp"
#include <numeric>
#include <algorithm>
#include <stdexcept>
#include <string>
#include <iostream>
#include <iomanip>

namespace USBCANBridge
{

// Private helper methods
uint8_t FixedSizeFrame::calculateChecksum() const {
	/**
	 * @brief Compute checksum over mutable protocol region.
	 * @details Sums bytes: type, frame_type, frame_fmt, id[4], dlc,
	 *          data[8], reserved. Returns low 8 bits.
	 * @return 8-bit checksum value.
	 * @complexity O(1) (fixed number of additions).
	 */
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

void FixedSizeFrame::validateIndex(std::size_t index) const {
	/**
	 * @brief Bounds check for random byte access.
	 * @param index Byte position (0-19 inclusive).
	 * @throw std::out_of_range if index >= 20.
	 */
	if (index >= 20) {
		throw std::out_of_range("Index " + std::to_string(index) +
		                        " out of range for FixedSizeFrame (0-19)");
	}
}

// Constructors
FixedSizeFrame::FixedSizeFrame()
	: AdapterBaseFrame(),
	msg_header_(to_uint8(Constants::MSG_HEADER)),
	reserved_(to_uint8(Constants::RESERVED0)) {
}

FixedSizeFrame::FixedSizeFrame(Type type, FrameType frame_type, FrameFmt fmt)
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
		throw std::invalid_argument("Invalid frame_type for FixedSizeFrame");
	}
	if (fmt != FrameFmt::DATA_FIXED && fmt != FrameFmt::REMOTE_FIXED) {
		throw std::invalid_argument("Invalid frame_fmt for FixedSizeFrame");
	}

	// Initialize arrays
	id_bytes_.fill(0);
	dlc_ = 0;
	data_.fill(0);
	checksum_ = calculateChecksum();
}

// Index access operators
uint8_t& FixedSizeFrame::operator[](std::size_t index) {
	/**
	 * @brief Direct unchecked byte access (except internal validateIndex call).
	 * @param index Byte offset (0..19).
	 * @return Reference to underlying byte for mutation.
	 * @throw std::out_of_range if index invalid.
	 */
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

const uint8_t& FixedSizeFrame::operator[](std::size_t index) const {
	/**
	 * @brief Const overload delegating to non-const implementation.
	 */
	return const_cast<FixedSizeFrame*>(this)->operator[](index);
}

uint8_t& FixedSizeFrame::at(std::size_t index) {
	/**
	 * @brief Bounds-checked mutable access (alias of operator[] here).
	 */
	return this->operator[](index);
}

const uint8_t& FixedSizeFrame::at(std::size_t index) const {
	/**
	 * @brief Bounds-checked const access (alias of operator[] here).
	 */
	return this->operator[](index);
}

// STL-style interface
uint8_t* FixedSizeFrame::begin() {
	/** @brief Iterator to first byte (start byte). */
	return &this->operator[](0);
}

uint8_t* FixedSizeFrame::end() {
	/** @brief Iterator one past last byte (offset 20). */
	return &this->operator[](0) + 20;
}

const uint8_t* FixedSizeFrame::begin() const {
	/** @brief Const iterator to first byte. */
	return &this->operator[](0);
}

const uint8_t* FixedSizeFrame::end() const {
	/** @brief Const iterator one past last byte. */
	return &this->operator[](0) + 20;
}

void FixedSizeFrame::fill(uint8_t value) {
	/**
	 * @brief Overwrite entire 20-byte frame with value.
	 * @warning Invalidates semantic meaning; usually for testing only.
	 */
	std::fill(this->begin(), this->end(), value);
}

std::size_t FixedSizeFrame::size() const {
	/** @brief Constant serialized size (20 bytes). */
	return 20;
}

// Data range validation
bool FixedSizeFrame::inDataRange(size_t index) const {
	/**
	 * @brief Check if index is within data payload array (0..7).
	 */
	return index <= 7;
}

// Data manipulation methods
void FixedSizeFrame::setData(const std::vector<uint8_t>& new_data) {
	/**
	 * @brief Set payload bytes (pads with zeros if fewer than 8).
	 * @param new_data Vector size 0..8.
	 * @throw std::invalid_argument if new_data larger than 8.
	 */
	if (new_data.size() > data_.size()) {
		throw std::invalid_argument("Data size exceeds fixed frame capacity");
	}

	std::size_t bytes_to_copy = std::min(new_data.size(), data_.size());
	std::copy(new_data.begin(), new_data.begin() + bytes_to_copy, data_.begin());

	if (bytes_to_copy < data_.size()) {
		std::fill(data_.begin() + bytes_to_copy, data_.end(), 0);
	}

	checksum_ = calculateChecksum();
}

void FixedSizeFrame::setData(size_t index, uint8_t value) {
	/**
	 * @brief Set a single payload byte.
	 * @param index 0..7
	 * @throw std::out_of_range if index invalid.
	 */
	if (!inDataRange(index)) {
		throw std::out_of_range("Data index out of range");
	}
	data_.at(index) = value;
	checksum_ = calculateChecksum();
}

std::vector<uint8_t> FixedSizeFrame::getData() const {
	/** @brief Return copy of entire 8-byte payload buffer. */
	return std::vector<uint8_t>(data_.begin(), data_.end());
}

uint8_t FixedSizeFrame::getData(size_t index) const {
	/**
	 * @brief Read single payload byte.
	 * @throw std::out_of_range if index invalid.
	 */
	if (!inDataRange(index)) {
		throw std::out_of_range("Data index out of range");
	}
	return data_.at(index);
}

// Protocol handling methods
void FixedSizeFrame::setType(Type type) {
	/**
	 * @brief Assign meta type (DATA_FIXED or CONF_FIXED) and refresh checksum.
	 * @throw std::invalid_argument if unsupported.
	 */
	if (type != Type::DATA_FIXED && type != Type::CONF_FIXED) {
		throw std::invalid_argument("Invalid type for FixedSizeFrame");
	}
	type_ = to_uint8(type);
	checksum_ = calculateChecksum();
}

Type FixedSizeFrame::getType() const {
	/** @brief Current meta type. */
	return static_cast<Type>(type_);
}

void FixedSizeFrame::setFrameType(FrameType frame_type) {
	/**
	 * @brief Assign identifier width variant and refresh checksum.
	 * @throw std::invalid_argument if unsupported.
	 */
	if (frame_type != FrameType::STD_FIXED && frame_type != FrameType::EXT_FIXED) {
		throw std::invalid_argument("Invalid frame type for FixedSizeFrame");
	}
	frame_type_ = to_uint8(frame_type);
	checksum_ = calculateChecksum();
}

FrameType FixedSizeFrame::getFrameType() const {
	/** @brief Current frame type. */
	return static_cast<FrameType>(frame_type_);
}

void FixedSizeFrame::setFrameFmt(FrameFmt frame_fmt) {
	/**
	 * @brief Assign frame format and refresh checksum.
	 * @throw std::invalid_argument if unsupported.
	 */
	if (frame_fmt != FrameFmt::DATA_FIXED && frame_fmt != FrameFmt::REMOTE_FIXED) {
		throw std::invalid_argument("Invalid frame format for FixedSizeFrame");
	}
	frame_fmt_ = to_uint8(frame_fmt);
	checksum_ = calculateChecksum();
}

FrameFmt FixedSizeFrame::getFrameFmt() const {
	/** @brief Current frame format. */
	return static_cast<FrameFmt>(frame_fmt_);
}

void FixedSizeFrame::setDLC(uint8_t dlc) {
	/**
	 * @brief Set DLC and refresh checksum.
	 * @throw std::invalid_argument if dlc > 8.
	 */
	if (dlc > 8) {
		throw std::invalid_argument("DLC cannot exceed 8 for FixedSizeFrame");
	}
	dlc_ = dlc;
	checksum_ = calculateChecksum();
}

uint8_t FixedSizeFrame::getDLC() const {
	/** @brief Current DLC value. */
	return dlc_;
}

bool FixedSizeFrame::isValidLength() const {
	/** @brief Check if DLC is within valid range. */
	return dlc_ <= 8;
}

void FixedSizeFrame::setID(uint32_t id_value) {
	/**
	 * @brief Set CAN identifier and refresh checksum.
	 * @throw std::invalid_argument on range violation.
	 */
	const FrameType frame_type = getFrameType();

	if (frame_type == FrameType::STD_FIXED) {
		if (id_value > 0x7FF) {
			throw std::invalid_argument("Standard ID exceeds 11-bit range");
		}
		// Store as little-endian in first 2 bytes
		id_bytes_[0] = static_cast<uint8_t>(id_value & 0xFF);
		id_bytes_[1] = static_cast<uint8_t>((id_value >> 8) & 0x07);
		id_bytes_[2] = 0;
		id_bytes_[3] = 0;
	} else {
		if (id_value > 0x1FFFFFFF) {
			throw std::invalid_argument("Extended ID exceeds 29-bit range");
		}
		// Store as little-endian in all 4 bytes
		id_bytes_[0] = static_cast<uint8_t>(id_value & 0xFF);
		id_bytes_[1] = static_cast<uint8_t>((id_value >> 8) & 0xFF);
		id_bytes_[2] = static_cast<uint8_t>((id_value >> 16) & 0xFF);
		id_bytes_[3] = static_cast<uint8_t>((id_value >> 24) & 0x1F);
	}
	checksum_ = calculateChecksum();
}

uint32_t FixedSizeFrame::getID() const {
	/** @brief Reconstruct ID from stored bytes. */
	const FrameType frame_type = getFrameType();

	if (frame_type == FrameType::STD_FIXED) {
		return static_cast<uint32_t>(id_bytes_[0]) |
		       (static_cast<uint32_t>(id_bytes_[1] & 0x07) << 8);
	} else {
		return static_cast<uint32_t>(id_bytes_[0]) |
		       (static_cast<uint32_t>(id_bytes_[1]) << 8) |
		       (static_cast<uint32_t>(id_bytes_[2]) << 16) |
		       (static_cast<uint32_t>(id_bytes_[3] & 0x1F) << 24);
	}
}

std::vector<uint8_t> FixedSizeFrame::getIDBytes() const {
	/** @brief Return copy of ID byte array. */
	return std::vector<uint8_t>(id_bytes_.begin(), id_bytes_.end());
}

void FixedSizeFrame::setIDBytes(const std::vector<uint8_t>& id_bytes) {
	/**
	 * @brief Set ID from byte vector and refresh checksum.
	 * @throw std::invalid_argument if size != 4.
	 */
	if (id_bytes.size() != 4) {
		throw std::invalid_argument("FixedSizeFrame requires exactly 4 ID bytes");
	}
	std::copy(id_bytes.begin(), id_bytes.end(), id_bytes_.begin());
	checksum_ = calculateChecksum();
}

bool FixedSizeFrame::isValidID() const {
	/** @brief Validate ID range for current frame type. */
	const FrameType frame_type = getFrameType();
	const uint32_t id_value = getID();

	if (frame_type == FrameType::STD_FIXED) {
		return id_value <= 0x7FF;
	} else {
		return id_value <= 0x1FFFFFFF;
	}
}

// Checksum methods
uint8_t FixedSizeFrame::getChecksum() const {
	/** @brief Current checksum value. */
	return checksum_;
}

void FixedSizeFrame::updateChecksum() {
	/** @brief Recalculate and update checksum. */
	checksum_ = calculateChecksum();
}

// Validation and serialization
bool FixedSizeFrame::isValidFrame() const {
	/** @brief Comprehensive frame validation. */
	return isValidLength() && isValidID() &&
	       (checksum_ == calculateChecksum());
}

std::vector<uint8_t> FixedSizeFrame::serialize() const {
	/** @brief Generate serialized 20-byte representation. */
	std::vector<uint8_t> result(20);
	for (std::size_t i = 0; i < 20; ++i) {
		result[i] = this->operator[](i);
	}
	return result;
}

bool FixedSizeFrame::deserialize(const std::vector<uint8_t>& raw_data) {
	/** @brief Parse 20-byte input into frame structure. */
	if (raw_data.size() != 20) {
		return false;
	}

	if (raw_data[0] != to_uint8(Constants::START_BYTE) ||
	    raw_data[1] != to_uint8(Constants::MSG_HEADER)) {
		return false;
	}

	// Copy data
	type_ = raw_data[2];
	frame_type_ = raw_data[3];
	frame_fmt_ = raw_data[4];
	std::copy(raw_data.begin() + 5, raw_data.begin() + 9, id_bytes_.begin());
	dlc_ = raw_data[9];
	std::copy(raw_data.begin() + 10, raw_data.begin() + 18, data_.begin());
	checksum_ = raw_data[19];

	// Validate
	return isValidFrame();
}

} // namespace USBCANBridge
