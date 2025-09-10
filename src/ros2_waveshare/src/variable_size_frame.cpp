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
#include <cstring>  // for std::memcpy

namespace USBCANBridge
{

// Constructors
VariableSizeFrame::VariableSizeFrame()
	: AdapterBaseFrame(),
	type_header_(to_uint8(Type::DATA_VAR)),
	frame_type_(to_uint8(FrameType::STD_VAR)),
	frame_fmt_(to_uint8(FrameFmt::DATA_VAR)),
	dlc_(0),
	end_byte_(to_uint8(Constants::END_BYTE)),
	id_size_(2) {
	// Initialize arrays to zero
	id_.fill(0);
	data_.fill(0);
}

VariableSizeFrame::VariableSizeFrame(FrameType frame_type, FrameFmt frame_fmt, uint8_t dlc)
	: AdapterBaseFrame(),
	type_header_(to_uint8(Type::DATA_VAR)),
	frame_type_(to_uint8(frame_type)),
	frame_fmt_(to_uint8(frame_fmt)),
	dlc_(dlc),
	end_byte_(to_uint8(Constants::END_BYTE)),
	id_size_((frame_type == FrameType::STD_VAR) ? 2 : 4) {

	// Validate parameters
	if (frame_type != FrameType::STD_VAR && frame_type != FrameType::EXT_VAR) {
		throw std::invalid_argument("Invalid frame type for VariableSizeFrame");
	}
	if (frame_fmt != FrameFmt::DATA_VAR && frame_fmt != FrameFmt::REMOTE_VAR) {
		throw std::invalid_argument("Invalid frame format for VariableSizeFrame");
	}
	if (dlc > 8) {
		throw std::invalid_argument("DLC must be 0-8 for CAN frames");
	}

	// Initialize arrays to zero
	id_.fill(0);
	data_.fill(0);
}

// Index access operators with lookup table for better performance
uint8_t& VariableSizeFrame::operator[](std::size_t index) {
	/**
	 * @brief Random access to frame bytes in serialized layout order.
	 * @param index 0 .. frame_end
	 * @details Uses lookup table for header fields (0-4) and range-based access
	 *          for ID and data sections to minimize branching.
	 */
	validateIndex(index);

	// Header fields: use array lookup for O(1) access without branching
	if (__builtin_expect(index < 5, 1)) {
		static uint8_t* const header_fields[] = {
			const_cast<uint8_t*>(&start_byte_),
			const_cast<uint8_t*>(&type_header_),
			&frame_type_,
			&frame_fmt_,
			&dlc_
		};
		return *header_fields[index];
	}

	// Calculate data start position once
	const std::size_t data_start = getDataStart();

	// ID and data sections: use range comparisons for efficient access
	if (__builtin_expect(index < data_start, 1)) {
		// ID bytes: index 5 to data_start-1
		return id_[index - 5];
	} else if (__builtin_expect(index < data_start + dlc_, 1)) {
		// Data bytes: data_start to data_start+dlc_-1
		return data_[index - data_start];
	} else {
		// End byte
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
	/** @brief Calculate total serialized frame size. */
	return 3 + id_size_ + dlc_; // start + type + end + id_bytes + data_bytes
}

// Data range validation
bool VariableSizeFrame::inDataRange(size_t index) const {
	/** @brief Check if index is within valid data payload range. */
	return index < dlc_;
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

	dlc_ = static_cast<uint8_t>(new_data.size());

	// Fast copy using iterators (better than element-by-element)
	std::copy(new_data.begin(), new_data.end(), data_.begin());

	// Clear remaining bytes if new data is smaller
	if (new_data.size() < MAX_DATA_SIZE) {
		std::fill(data_.begin() + new_data.size(), data_.end(), 0);
	}
}

void VariableSizeFrame::setData(size_t index, uint8_t value) {
	/**
	 * @brief Modify single payload byte value.
	 * @throw std::out_of_range if index invalid.
	 */
	if (!inDataRange(index)) {
		throw std::out_of_range("Data index " + std::to_string(index) + " out of range");
	}
	data_[index] = value;
}

uint8_t VariableSizeFrame::getData(size_t index) const {
	/** @brief Fast payload byte access by index. */
	if (!inDataRange(index)) {
		throw std::out_of_range("Data index " + std::to_string(index) + " out of range");
	}
	return data_[index];
}

// Data access methods
std::pair<std::array<uint8_t, 8>, uint8_t> VariableSizeFrame::getDataArray() const {
	/** @brief Return data as array with actual size (zero-copy). */
	return std::make_pair(data_, dlc_);
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
	 * @brief Change frame type and adjust ID field size.
	 * @throw std::invalid_argument on unsupported value.
	 */
	if (frame_type != FrameType::STD_VAR && frame_type != FrameType::EXT_VAR) {
		throw std::invalid_argument("Invalid frame type for VariableSizeFrame");
	}

	frame_type_ = to_uint8(frame_type);
	const uint8_t new_id_size = (frame_type == FrameType::STD_VAR) ? 2 : 4;

	// Clear additional bytes if switching from extended to standard
	if (new_id_size < id_size_) {
		std::fill(id_.begin() + new_id_size, id_.begin() + id_size_, 0);
	}

	id_size_ = new_id_size;
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
	 * @brief Set DLC with validation.
	 * @throw std::invalid_argument if dlc > 8.
	 */
	if (dlc > 8) {
		throw std::invalid_argument("DLC must be 0-8 for CAN frames");
	}
	dlc_ = dlc;
}

uint8_t VariableSizeFrame::getDLC() const {
	/** @brief Current DLC value. */
	return dlc_;
}

bool VariableSizeFrame::isValidLength() const {
	/** @brief Fast length consistency check. */
	return dlc_ <= 8;
}

void VariableSizeFrame::setID(uint32_t id_value) {
	/**
	 * @brief Set CAN identifier with range validation.
	 * @param id_value 11-bit or 29-bit value.
	 * @throw std::invalid_argument on range or type mismatch.
	 */
	const FrameType frame_type = getFrameType();

	if (frame_type == FrameType::STD_VAR) {
		if (id_value > 0x7FF) {
			throw std::invalid_argument("Standard ID must be in range 0 to 2047 (0x7FF)");
		}
		id_size_ = 2;
		// Use memcpy for efficient byte-level copying
		std::memcpy(id_.data(), &id_value, 2);
		// Clear unused bytes
		std::fill(id_.begin() + 2, id_.end(), 0);
	} else if (frame_type == FrameType::EXT_VAR) {
		if (id_value > 0x1FFFFFFF) {
			throw std::invalid_argument("Extended ID must be in range 0 to 536870911 (0x1FFFFFFF)");
		}
		id_size_ = 4;
		// Use memcpy for efficient byte-level copying
		std::memcpy(id_.data(), &id_value, 4);
	} else {
		throw std::invalid_argument("Invalid frame type for setting ID");
	}
}

uint32_t VariableSizeFrame::getID() const {
	/** @brief Reconstruct numeric identifier from stored bytes. */
	uint32_t id_value = 0;
	// Use memcpy for efficient reconstruction, handling endianness
	std::memcpy(&id_value, id_.data(), id_size_);
	return id_value;
}

void VariableSizeFrame::setIDBytes(const std::vector<uint8_t>& id_bytes) {
	/**
	 * @brief Assign ID bytes with size enforcement.
	 * @throw std::invalid_argument if size mismatch.
	 */
	const FrameType frame_type = getFrameType();

	if (frame_type == FrameType::STD_VAR && id_bytes.size() != 2) {
		throw std::invalid_argument("ID must be exactly 2 bytes for standard frame");
	}
	if (frame_type == FrameType::EXT_VAR && id_bytes.size() != 4) {
		throw std::invalid_argument("ID must be exactly 4 bytes for extended frame");
	}

	id_size_ = static_cast<uint8_t>(id_bytes.size());
	// Fast copy using STL algorithm
	std::copy(id_bytes.begin(), id_bytes.end(), id_.begin());

	// Clear remaining bytes if needed
	if (id_bytes.size() < MAX_ID_SIZE) {
		std::fill(id_.begin() + id_bytes.size(), id_.end(), 0);
	}
}

// ID access methods
std::pair<std::array<uint8_t, 4>, uint8_t> VariableSizeFrame::getIDArray() const {
	/** @brief Return ID as array with actual size (zero-copy). */
	return std::make_pair(id_, id_size_);
}

bool VariableSizeFrame::isValidID() const {
	/** @brief Fast ID size + numeric range validation. */
	const FrameType frame_type = getFrameType();
	const uint32_t id_value = getID();

	if (frame_type == FrameType::STD_VAR) {
		return id_value <= 0x7FF && id_size_ == 2;
	} else if (frame_type == FrameType::EXT_VAR) {
		return id_value <= 0x1FFFFFFF && id_size_ == 4;
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
	 * @brief Serialize frame to byte buffer for transmission.
	 * @throws std::invalid_argument if frame invalid.
	 */
	if (!isValidFrame()) {
		throw std::invalid_argument("Cannot serialize an invalid frame");
	}

	const std::size_t total_size = size();
	std::vector<uint8_t> result;
	result.reserve(total_size); // Pre-allocate to avoid reallocations

	result.push_back(start_byte_);

	// Construct type byte from individual components
	uint8_t type_byte = make_variable_type_byte(getFrameType(), getFrameFmt(), getDLC());
	result.push_back(type_byte);

	// Bulk insert operations for better performance
	result.insert(result.end(), id_.begin(), id_.begin() + id_size_);
	result.insert(result.end(), data_.begin(), data_.begin() + dlc_);
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

	// Update sizes and clear arrays
	id_size_ = static_cast<uint8_t>(expected_id_size);
	dlc_ = static_cast<uint8_t>(expected_data_size);
	id_.fill(0);
	data_.fill(0);

	// Copy ID and data bytes efficiently
	std::copy(raw_data.begin() + 2, raw_data.begin() + 2 + expected_id_size, id_.begin());
	std::copy(raw_data.begin() + 2 + expected_id_size,
	          raw_data.begin() + 2 + expected_id_size + expected_data_size, data_.begin());

	return isValidFrame();
}

} // namespace USBCANBridge
