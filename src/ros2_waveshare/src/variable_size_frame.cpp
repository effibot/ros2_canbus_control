/**
 * @file variable_size_frame.cpp
 * @brief Implementation of Variable length frame class for USB-CAN adapter
 *
 * @author Andrea Efficace
 * @date September 2025
 */

#include "ros2_waveshare/variable_size_frame.hpp"
#include <algorithm>
#include <stdexcept>
#include <string>

namespace USBCANBridge
{

void VariableSizeFrame::updateIndices() {
	/**
	 * @brief Recompute dynamic boundary indices after ID/data resize.
	 * @details ID_END_ points to last ID byte; DATA_START_ first data byte;
	 *          DATA_END_ last data byte; FRAME_END_ index of end marker.
	 */
	ID_END_ = static_cast<VarSizeIndex>(static_cast<int>(VarSizeIndex::ID_START) + id_.size() - 1);
	DATA_START_ = static_cast<VarSizeIndex>(static_cast<int>(ID_END_) + 1);
	DATA_END_ = static_cast<VarSizeIndex>(static_cast<int>(DATA_START_) + data_.size() - 1);
	FRAME_END_ = static_cast<VarSizeIndex>(static_cast<int>(DATA_END_) + 1);
}

void VariableSizeFrame::validateIndex(std::size_t index) const {
	/**
	 * @brief Validate random-access index.
	 * @throws std::out_of_range if beyond FRAME_END_.
	 */
	if (index > to_uint(FRAME_END_)) {
		throw std::out_of_range("Index " + std::to_string(index) +
		                        " out of range for VariableSizeFrame");
	}
}

// Constructors
VariableSizeFrame::VariableSizeFrame()
	: AdapterBaseFrame(),
	type_header_(to_uint8(Type::DATA_VAR)),
	end_byte_(to_uint8(Constants::END_BYTE)) {
}

VariableSizeFrame::VariableSizeFrame(FrameType frame_type, FrameFmt frame_fmt, uint8_t dlc)
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
	} else  {
		id_.resize(4, 0);
	}

	// Initialize data vector based on DLC
	data_.resize(dlc, 0);

	updateIndices();
}

// Index access operators
uint8_t& VariableSizeFrame::operator[](std::size_t index) {
	/**
	 * @brief Random access to serialized layout view.
	 * @param index 0 .. FRAME_END_
	 * @throw std::out_of_range if invalid.
	 */
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
			return id_[index - 5];
		}
		if (index < to_uint(FRAME_END_)) {
			return data_[index - to_uint(DATA_START_)];
		}
		return const_cast<uint8_t&>(end_byte_);
	}
}

const uint8_t& VariableSizeFrame::operator[](std::size_t index) const {
	/** @brief Const overload delegating to non-const. */
	return const_cast<VariableSizeFrame*>(this)->operator[](index);
}

uint8_t& VariableSizeFrame::at(std::size_t index) {
	/** @brief Bounds-checked mutable access (alias). */
	return this->operator[](index);
}

const uint8_t& VariableSizeFrame::at(std::size_t index) const {
	/** @brief Bounds-checked const access (alias). */
	return this->operator[](index);
}

// STL-style interface
uint8_t* VariableSizeFrame::begin() {
	/** @brief Iterator to first byte. */
	return &this->operator[](0);
}

uint8_t* VariableSizeFrame::end() {
	/** @brief Iterator one past last byte. */
	return &this->operator[](0) + size();
}

const uint8_t* VariableSizeFrame::begin() const {
	/** @brief Const iterator to first byte. */
	return &this->operator[](0);
}

const uint8_t* VariableSizeFrame::end() const {
	/** @brief Const iterator one past last byte. */
	return &this->operator[](0) + size();
}

void VariableSizeFrame::fill(uint8_t value) {
	/** @brief Overwrite all bytes (used mainly for tests). */
	std::fill(this->begin(), this->end(), value);
}

std::size_t VariableSizeFrame::size() const {
	/** @brief Serialized size (dynamic). */
	size_t constants_size = 3; // start byte + type byte + end byte
	return constants_size + id_.size() + data_.size();
}

// Data range validation
bool VariableSizeFrame::inDataRange(size_t index) const {
	/** @brief Check if index in payload range. */
	return index < data_.size();
}

// Data manipulation methods
void VariableSizeFrame::setData(const std::vector<uint8_t>& new_data) {
	/**
	 * @brief Replace payload with vector up to 8 bytes.
	 * @throw std::invalid_argument if size > 8.
	 */
	if (new_data.size() > 8) {
		throw std::invalid_argument("Data size " + std::to_string(new_data.size()) +
		                            " exceeds maximum of 8");
	}

	data_ = new_data;
	dlc_ = static_cast<uint8_t>(data_.size());
	updateIndices();
}

void VariableSizeFrame::setData(size_t index, uint8_t value) {
	/**
	 * @brief Mutate single payload byte.
	 * @throw std::out_of_range if index invalid.
	 */
	if (!inDataRange(index)) {
		throw std::out_of_range("Data index " + std::to_string(index) + " out of range");
	}
	data_.at(index) = value;
}

std::vector<uint8_t> VariableSizeFrame::getData() const {
	/** @brief Copy of current payload. */
	return data_;
}

uint8_t VariableSizeFrame::getData(size_t index) const {
	/** @brief Read payload byte by index. */
	if (!inDataRange(index)) {
		throw std::out_of_range("Data index " + std::to_string(index) + " out of range");
	}
	return data_.at(index);
}

// Protocol handling methods
void VariableSizeFrame::setType(Type type) {
	/**
	 * @brief Enforce DATA_VAR type (other values rejected).
	 * @throw std::invalid_argument if type not DATA_VAR.
	 */
	if (type != Type::DATA_VAR) {
		throw std::invalid_argument("Invalid type for VariableSizeFrame - must be DATA_VAR");
	}
	// Type is always DATA_VAR for variable frames
}

Type VariableSizeFrame::getType() const {
	/** @brief Effective variable frame meta type. */
	return static_cast<Type>(type_header_);
}

void VariableSizeFrame::setFrameType(FrameType frame_type) {
	/**
	 * @brief Change frame type; adjusts ID size and indices.
	 * @throw std::invalid_argument on unsupported value.
	 */
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

FrameType VariableSizeFrame::getFrameType() const {
	/** @brief Current variable frame ID width. */
	return static_cast<FrameType>(frame_type_);
}

void VariableSizeFrame::setFrameFmt(FrameFmt frame_fmt) {
	/**
	 * @brief Set frame format (data or remote).
	 * @throw std::invalid_argument if unsupported value.
	 */
	if (frame_fmt != FrameFmt::DATA_VAR && frame_fmt != FrameFmt::REMOTE_VAR) {
		throw std::invalid_argument("Invalid frame format for VariableSizeFrame");
	}
	frame_fmt_ = to_uint8(frame_fmt);
}

FrameFmt VariableSizeFrame::getFrameFmt() const {
	/** @brief Current variable frame format. */
	return static_cast<FrameFmt>(frame_fmt_);
}

void VariableSizeFrame::setDLC(uint8_t dlc) {
	/**
	 * @brief Set DLC (does not resize payload to avoid data loss).
	 * @throw std::invalid_argument if dlc > 8.
	 */
	if (dlc > 8) {
		throw std::invalid_argument("DLC must be 0-8 for CAN frames");
	}
	dlc_ = dlc;
	// Note: This doesn't resize data vector to prevent data loss
	// Use setData() to change actual data size
}

uint8_t VariableSizeFrame::getDLC() const {
	/** @brief Current DLC value. */
	return dlc_;
}

bool VariableSizeFrame::isValidLength() const {
	/** @brief Validate payload length consistency (dlc == data_.size()). */
	return dlc_ <= 8 && dlc_ == data_.size();
}

void VariableSizeFrame::setID(uint32_t id_value) {
	/**
	 * @brief Assign CAN identifier adjusting ID vector size as needed.
	 * @param id_value 11-bit or 29-bit value.
	 * @throw std::invalid_argument on range or type mismatch.
	 */
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

uint32_t VariableSizeFrame::getID() const {
	/** @brief Reconstruct numeric identifier from stored bytes. */
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

std::vector<uint8_t> VariableSizeFrame::getIDBytes() const {
	/** @brief Return copy of ID byte sequence. */
	return id_;
}

void VariableSizeFrame::setIDBytes(const std::vector<uint8_t>& id_bytes) {
	/**
	 * @brief Bulk assign ID bytes enforcing expected size per frame type.
	 * @throw std::invalid_argument if size mismatch.
	 */
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

bool VariableSizeFrame::isValidID() const {
	/** @brief Validate ID size + numeric range for type. */
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
bool VariableSizeFrame::isValidFrame() const {
	/**
	 * @brief Comprehensive structural validation including start/end markers.
	 */
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

std::vector<uint8_t> VariableSizeFrame::serialize() const {
	/**
	 * @brief Serialize dynamic frame to byte vector.
	 * @throws std::invalid_argument if frame invalid.
	 */
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

bool VariableSizeFrame::deserialize(const std::vector<uint8_t>& raw_data) {
	/**
	 * @brief Parse raw variable frame bytes into object state.
	 * @return true if successful & valid; false otherwise.
	 */
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

} // namespace USBCANBridge
