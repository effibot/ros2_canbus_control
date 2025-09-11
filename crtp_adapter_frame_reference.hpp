/**
 * @file crtp_adapter_frame_reference.hpp
 * @brief Complete CRTP implementation reference for USB-CAN adapter frames
 *
 * This file demonstrates a complete CRTP-based implementation with:
 * - Zero runtime overhead polymorphism
 * - Smart return types for debugging
 * - Expressive constants and enums
 * - Stack-allocated storage using std::array
 * - Compile-time type selection via traits
 *
 * @author Andrea Efficace
 * @date September 2025
 */

#pragma once

#include <array>
#include <vector>
#include <cstdint>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <type_traits>

namespace USBCANBridge_CRTP {

// ================================================================================================
// PROTOCOL CONSTANTS AND ENUMS (same as original)
// ================================================================================================

enum class Constants : uint8_t {
	START_BYTE = 0xAA,
	MSG_HEADER = 0x55,
	END_BYTE = 0x55,
	RESERVED0 = 0x00
};

enum class Type : uint8_t {
	DATA_FIXED = 0x01,
	DATA_VAR = 0xC0,
	CONF_FIXED = 0x02,
	CONF_VAR = 0x12
};

enum class FrameType : uint8_t {
	STD_FIXED = 0x01,
	STD_VAR = 0,
	EXT_FIXED = 0x02,
	EXT_VAR = 1
};

enum class FrameFmt : uint8_t {
	DATA_FIXED = 0x01,
	REMOTE_FIXED = 0x02,
	DATA_VAR = 0,
	REMOTE_VAR = 1
};

enum class FixedSizeIndex : std::size_t {
	START = 0, HEADER = 1, TYPE = 2, FRAME_TYPE = 3, FRAME_FMT = 4,
	ID_0 = 5, ID_1 = 6, ID_2 = 7, ID_3 = 8, DLC = 9,
	DATA_0 = 10, DATA_1 = 11, DATA_2 = 12, DATA_3 = 13,
	DATA_4 = 14, DATA_5 = 15, DATA_6 = 16, DATA_7 = 17,
	RESERVED = 18, CHECKSUM = 19
};

// Conversion helpers
constexpr uint8_t to_uint8(Constants v) {
	return static_cast<uint8_t>(v);
}
constexpr uint8_t to_uint8(Type v) {
	return static_cast<uint8_t>(v);
}
constexpr uint8_t to_uint8(FrameType v) {
	return static_cast<uint8_t>(v);
}
constexpr uint8_t to_uint8(FrameFmt v) {
	return static_cast<uint8_t>(v);
}
constexpr std::size_t to_uint(FixedSizeIndex v) {
	return static_cast<std::size_t>(v);
}

// ================================================================================================
// SMART RETURN TYPES AND DEBUG CONSTANTS
// ================================================================================================

namespace DebugConstants {
enum class ValidationResult : uint8_t {
	VALID = 0x00,
	INVALID_START_BYTE = 0x01,
	INVALID_HEADER = 0x02,
	INVALID_CHECKSUM = 0x03,
	INVALID_DLC = 0x04,
	INVALID_ID_RANGE = 0x05,
	INVALID_FRAME_SIZE = 0x06
};

enum class DebugLevel : uint8_t {
	NONE = 0, ERROR = 1, WARNING = 2, INFO = 3, VERBOSE = 4
};

enum class ProcessingState : uint8_t {
	UNINITIALIZED = 0x00,
	INITIALIZED = 0x01,
	SERIALIZED = 0x02,
	DESERIALIZED = 0x03,
	VALIDATED = 0x04,
	ERROR_STATE = 0xFF
};
}

enum class FrameOperationStatus : uint8_t {
	SUCCESS = 0x00,
	INVALID_DLC = 0x01,
	INVALID_ID = 0x02,
	INVALID_CHECKSUM = 0x03,
	BUFFER_OVERFLOW = 0x04,
	INVALID_FRAME_TYPE = 0x05,
	SERIALIZATION_ERROR = 0x06
};

// Smart result wrapper
template<typename T>
struct FrameResult {
	T value;
	FrameOperationStatus status;
	const char* debug_message;

	bool is_success() const {
		return status == FrameOperationStatus::SUCCESS;
	}
	bool is_error() const {
		return !is_success();
	}
	explicit operator bool() const {
		return is_success();
	}

	static FrameResult success(T val) {
		return {std::move(val), FrameOperationStatus::SUCCESS, "Operation successful"};
	}

	static FrameResult error(FrameOperationStatus err, const char* msg) {
		return {T{}, err, msg};
	}
};

// Validation result with detailed info
struct ValidationResult {
	bool is_valid;
	DebugConstants::ValidationResult error_code;
	std::size_t error_byte_index;
	const char* description;

	static ValidationResult success() {
		return {true, DebugConstants::ValidationResult::VALID, 0, "Frame is valid"};
	}

	static ValidationResult invalid_checksum(std::size_t byte_idx) {
		return {false, DebugConstants::ValidationResult::INVALID_CHECKSUM,
		        byte_idx, "Checksum validation failed"};
	}

	static ValidationResult invalid_dlc(std::size_t byte_idx) {
		return {false, DebugConstants::ValidationResult::INVALID_DLC,
		        byte_idx, "DLC out of range (0-8)"};
	}

	static ValidationResult invalid_start_byte() {
		return {false, DebugConstants::ValidationResult::INVALID_START_BYTE,
		        0, "Invalid start byte"};
	}
};

// Expressive type aliases
namespace FrameTypes {
using CANIdentifier = uint32_t;
using DataLengthCode = uint8_t;
using ChecksumValue = uint8_t;
using CANPayload = std::array<uint8_t, 8>;
using CANPayloadPair = std::pair<CANPayload, DataLengthCode>;
using IDArray = std::array<uint8_t, 4>;
using IDArrayPair = std::pair<IDArray, uint8_t>;

struct FrameDebugInfo {
	DebugConstants::ProcessingState state;
	std::chrono::steady_clock::time_point created_at;
	std::chrono::steady_clock::time_point last_modified;
	uint32_t operation_count;
	ValidationResult last_validation;
};
}

// Serialization result with debug info
struct SerializationResult {
	std::vector<uint8_t> data;
	FrameOperationStatus status;
	std::size_t bytes_written;
	std::chrono::microseconds serialization_time;

	struct DebugInfo {
		std::size_t header_bytes;
		std::size_t id_bytes;
		std::size_t payload_bytes;
		std::size_t checksum_bytes;
		FrameTypes::ChecksumValue calculated_checksum;
	} debug_info;

	bool is_success() const {
		return status == FrameOperationStatus::SUCCESS;
	}
};

// ================================================================================================
// FRAME TRAITS (COMPILE-TIME TYPE SELECTION)
// ================================================================================================

// Forward declarations
class FixedSizeFrame;
class VariableSizeFrame;

template<typename Frame>
struct FrameTraits;

template<>
struct FrameTraits<FixedSizeFrame> {
	static constexpr std::size_t FRAME_SIZE = 20;
	static constexpr std::size_t ID_SIZE = 4;
	static constexpr std::size_t DATA_SIZE = 8;
	static constexpr bool IS_VARIABLE_SIZE = false;

	using StorageType = std::array<uint8_t, FRAME_SIZE>;
	using IDType = std::array<uint8_t, ID_SIZE>;
	using DataType = std::array<uint8_t, DATA_SIZE>;
	using SerializeReturnType = std::array<uint8_t, FRAME_SIZE>;
};

template<>
struct FrameTraits<VariableSizeFrame> {
	static constexpr std::size_t MAX_FRAME_SIZE = 13; // 1+1+4+8-1
	static constexpr std::size_t MAX_ID_SIZE = 4;
	static constexpr std::size_t MAX_DATA_SIZE = 8;
	static constexpr bool IS_VARIABLE_SIZE = true;

	using StorageType = std::array<uint8_t, MAX_FRAME_SIZE>;
	using IDType = std::array<uint8_t, MAX_ID_SIZE>;
	using DataType = std::array<uint8_t, MAX_DATA_SIZE>;
	using SerializeReturnType = std::vector<uint8_t>;
};

// Compile-time debug configuration
namespace CompileTimeDebug {
constexpr bool ENABLE_FRAME_VALIDATION = true;
constexpr bool ENABLE_PERFORMANCE_COUNTERS = true;
constexpr bool ENABLE_DEBUG_LOGGING = true;
constexpr DebugConstants::DebugLevel CURRENT_DEBUG_LEVEL = DebugConstants::DebugLevel::INFO;

template<DebugConstants::DebugLevel Level>
constexpr bool should_log() {
	return static_cast<int>(Level) <= static_cast<int>(CURRENT_DEBUG_LEVEL);
}
}

// ================================================================================================
// CRTP BASE CLASS
// ================================================================================================

template<typename Derived>
class AdapterBaseFrame {
public:
// Bring traits into scope
using traits = FrameTraits<Derived>;
using StorageType = typename traits::StorageType;
using SerializeReturnType = typename traits::SerializeReturnType;

private:
// CRTP access methods
Derived& derived() {
	return static_cast<Derived&>(*this);
}
const Derived& derived() const {
	return static_cast<const Derived&>(*this);
}

protected:
const uint8_t start_byte_;
mutable FrameTypes::FrameDebugInfo debug_info_{};

public:
// Constructor
explicit AdapterBaseFrame() : start_byte_(to_uint8(Constants::START_BYTE)) {
	debug_info_.state = DebugConstants::ProcessingState::INITIALIZED;
	debug_info_.created_at = std::chrono::steady_clock::now();
	debug_info_.last_modified = debug_info_.created_at;
	debug_info_.operation_count = 0;
}

// Disable copy/assignment to prevent slicing
AdapterBaseFrame(const AdapterBaseFrame&) = delete;
AdapterBaseFrame& operator=(const AdapterBaseFrame&) = delete;

// Enable move semantics
AdapterBaseFrame(AdapterBaseFrame&&) = default;
AdapterBaseFrame& operator=(AdapterBaseFrame&&) = default;
virtual ~AdapterBaseFrame() = default;

// ============================================================================================
// ARRAY-LIKE ACCESS INTERFACE (CRTP DELEGATION)
// ============================================================================================

uint8_t& operator[](std::size_t index) {
	increment_operation_counter();
	return derived().impl_subscript(index);
}

const uint8_t& operator[](std::size_t index) const {
	return derived().impl_subscript(index);
}

uint8_t& at(std::size_t index) {
	increment_operation_counter();
	return derived().impl_at(index);
}

const uint8_t& at(std::size_t index) const {
	return derived().impl_at(index);
}

// STL-style iterators
uint8_t* begin() {
	return derived().impl_begin();
}
uint8_t* end() {
	return derived().impl_end();
}
const uint8_t* begin() const {
	return derived().impl_begin();
}
const uint8_t* end() const {
	return derived().impl_end();
}

void fill(uint8_t value) {
	increment_operation_counter();
	derived().impl_fill(value);
}

constexpr std::size_t size() const {
	return derived().impl_size();
}

// ============================================================================================
// SMART DATA ACCESS INTERFACE
// ============================================================================================

FrameResult<FrameTypes::CANPayloadPair> getDataArray() const {
	auto validation = validateDataSection();
	if (!validation.is_valid) {
		return FrameResult<FrameTypes::CANPayloadPair>::error(
			FrameOperationStatus::INVALID_DLC, validation.description);
	}
	return derived().impl_getDataArray();
}

FrameResult<bool> setData(const FrameTypes::CANPayload& new_data, FrameTypes::DataLengthCode dlc) {
	increment_operation_counter();
	if (dlc > 8) {
		return FrameResult<bool>::error(FrameOperationStatus::INVALID_DLC, "DLC > 8");
	}
	return derived().impl_setData(new_data, dlc);
}

FrameResult<uint8_t> getData(std::size_t index) const {
	if (!inDataRange(index)) {
		return FrameResult<uint8_t>::error(FrameOperationStatus::BUFFER_OVERFLOW, "Index out of data range");
	}
	return derived().impl_getData(index);
}

bool inDataRange(std::size_t index) const {
	return derived().impl_inDataRange(index);
}

// ============================================================================================
// SMART PROTOCOL INTERFACE
// ============================================================================================

FrameResult<Type> getType() const {
	auto validation = validateFrame();
	if (!validation.is_valid) {
		return FrameResult<Type>::error(FrameOperationStatus::INVALID_FRAME_TYPE, validation.description);
	}
	return derived().impl_getType();
}

FrameResult<bool> setType(Type type) {
	increment_operation_counter();
	return derived().impl_setType(type);
}

FrameResult<FrameType> getFrameType() const {
	return derived().impl_getFrameType();
}

FrameResult<bool> setFrameType(FrameType frame_type) {
	increment_operation_counter();
	return derived().impl_setFrameType(frame_type);
}

FrameResult<FrameFmt> getFrameFmt() const {
	return derived().impl_getFrameFmt();
}

FrameResult<bool> setFrameFmt(FrameFmt frame_fmt) {
	increment_operation_counter();
	return derived().impl_setFrameFmt(frame_fmt);
}

FrameResult<FrameTypes::DataLengthCode> getDLC() const {
	return derived().impl_getDLC();
}

FrameResult<bool> setDLC(FrameTypes::DataLengthCode dlc) {
	increment_operation_counter();
	if (dlc > 8) {
		return FrameResult<bool>::error(FrameOperationStatus::INVALID_DLC, "DLC > 8");
	}
	return derived().impl_setDLC(dlc);
}

FrameResult<FrameTypes::CANIdentifier> getID() const {
	auto validation = validateIDSection();
	if (!validation.is_valid) {
		return FrameResult<FrameTypes::CANIdentifier>::error(
			FrameOperationStatus::INVALID_ID, validation.description);
	}
	return derived().impl_getID();
}

FrameResult<bool> setID(FrameTypes::CANIdentifier id) {
	increment_operation_counter();
	return derived().impl_setID(id);
}

FrameResult<FrameTypes::IDArrayPair> getIDArray() const {
	return derived().impl_getIDArray();
}

// ============================================================================================
// SMART SERIALIZATION INTERFACE
// ============================================================================================

SerializationResult serialize() const {
	auto start_time = std::chrono::steady_clock::now();
	auto result = derived().impl_serialize();
	auto end_time = std::chrono::steady_clock::now();

	result.serialization_time =
		std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

	debug_info_.state = DebugConstants::ProcessingState::SERIALIZED;
	debug_info_.last_modified = end_time;

	return result;
}

FrameResult<bool> deserialize(const std::vector<uint8_t>& data) {
	increment_operation_counter();
	auto result = derived().impl_deserialize(data);
	if (result.is_success()) {
		debug_info_.state = DebugConstants::ProcessingState::DESERIALIZED;
		debug_info_.last_modified = std::chrono::steady_clock::now();
	}
	return result;
}

// ============================================================================================
// VALIDATION AND DEBUG INTERFACE
// ============================================================================================

ValidationResult validateFrame() const {
	auto result = derived().impl_validateFrame();
	debug_info_.last_validation = result;
	if (result.is_valid) {
		debug_info_.state = DebugConstants::ProcessingState::VALIDATED;
	} else {
		debug_info_.state = DebugConstants::ProcessingState::ERROR_STATE;
	}
	return result;
}

ValidationResult validateDataSection() const {
	return derived().impl_validateDataSection();
}

ValidationResult validateIDSection() const {
	return derived().impl_validateIDSection();
}

const FrameTypes::FrameDebugInfo& getDebugInfo() const {
	return debug_info_;
}

// ============================================================================================
// EXPRESSIVE INTERFACE METHODS
// ============================================================================================

bool isDataFrame() const {
	auto fmt = getFrameFmt();
	return fmt.is_success() &&
	       (fmt.value == FrameFmt::DATA_FIXED || fmt.value == FrameFmt::DATA_VAR);
}

bool isRemoteFrame() const {
	auto fmt = getFrameFmt();
	return fmt.is_success() &&
	       (fmt.value == FrameFmt::REMOTE_FIXED || fmt.value == FrameFmt::REMOTE_VAR);
}

bool isExtendedFrame() const {
	auto type = getFrameType();
	return type.is_success() &&
	       (type.value == FrameType::EXT_FIXED || type.value == FrameType::EXT_VAR);
}

bool isStandardFrame() const {
	auto type = getFrameType();
	return type.is_success() &&
	       (type.value == FrameType::STD_FIXED || type.value == FrameType::STD_VAR);
}

bool isFixedSizeProtocol() const {
	if constexpr (traits::IS_VARIABLE_SIZE) {
		return false;
	} else {
		return true;
	}
}

bool isVariableSizeProtocol() const {
	return traits::IS_VARIABLE_SIZE;
}

// ============================================================================================
// COMPILE-TIME FEATURES
// ============================================================================================

template<DebugConstants::DebugLevel Level>
void debug_log(const char* message) const {
	if constexpr (CompileTimeDebug::ENABLE_DEBUG_LOGGING) {
		if constexpr (CompileTimeDebug::should_log<Level>()) {
			std::cout << "[" << static_cast<int>(Level) << "] " << message << std::endl;
		}
	}
}

constexpr std::size_t max_frame_size() const {
	if constexpr (traits::IS_VARIABLE_SIZE) {
		return traits::MAX_FRAME_SIZE;
	} else {
		return traits::FRAME_SIZE;
	}
}

private:
void increment_operation_counter() const {
	if constexpr (CompileTimeDebug::ENABLE_PERFORMANCE_COUNTERS) {
		++debug_info_.operation_count;
		debug_info_.last_modified = std::chrono::steady_clock::now();
	}
}
};

// ================================================================================================
// FIXED SIZE FRAME IMPLEMENTATION
// ================================================================================================

class FixedSizeFrame : public AdapterBaseFrame<FixedSizeFrame> {
private:
alignas(4) traits::StorageType frame_data_{};

public:
FixedSizeFrame() {
	// Initialize frame structure
	frame_data_[to_uint(FixedSizeIndex::START)] = start_byte_;
	frame_data_[to_uint(FixedSizeIndex::HEADER)] = to_uint8(Constants::MSG_HEADER);
	frame_data_[to_uint(FixedSizeIndex::TYPE)] = to_uint8(Type::DATA_FIXED);
	frame_data_[to_uint(FixedSizeIndex::FRAME_TYPE)] = to_uint8(FrameType::STD_FIXED);
	frame_data_[to_uint(FixedSizeIndex::FRAME_FMT)] = to_uint8(FrameFmt::DATA_FIXED);
	frame_data_[to_uint(FixedSizeIndex::RESERVED)] = to_uint8(Constants::RESERVED0);
	updateChecksum();
}

// ============================================================================================
// IMPLEMENTATION METHODS (CALLED BY CRTP BASE)
// ============================================================================================

uint8_t& impl_subscript(std::size_t index) {
	return frame_data_[index];
}

const uint8_t& impl_subscript(std::size_t index) const {
	return frame_data_[index];
}

uint8_t& impl_at(std::size_t index) {
	if (index >= frame_data_.size()) {
		throw std::out_of_range("Index out of range");
	}
	return frame_data_[index];
}

const uint8_t& impl_at(std::size_t index) const {
	if (index >= frame_data_.size()) {
		throw std::out_of_range("Index out of range");
	}
	return frame_data_[index];
}

uint8_t* impl_begin() {
	return frame_data_.data();
}
uint8_t* impl_end() {
	return frame_data_.data() + frame_data_.size();
}
const uint8_t* impl_begin() const {
	return frame_data_.data();
}
const uint8_t* impl_end() const {
	return frame_data_.data() + frame_data_.size();
}

void impl_fill(uint8_t value) {
	frame_data_.fill(value);
	updateChecksum();
}

constexpr std::size_t impl_size() const {
	return traits::FRAME_SIZE;
}

// Data access implementations
FrameResult<FrameTypes::CANPayloadPair> impl_getDataArray() const {
	FrameTypes::CANPayload data_array{};
	auto dlc = frame_data_[to_uint(FixedSizeIndex::DLC)];

	std::copy(frame_data_.begin() + to_uint(FixedSizeIndex::DATA_0),
	          frame_data_.begin() + to_uint(FixedSizeIndex::DATA_0) + 8,
	          data_array.begin());

	return FrameResult<FrameTypes::CANPayloadPair>::success({data_array, dlc});
}

FrameResult<bool> impl_setData(const FrameTypes::CANPayload& new_data, FrameTypes::DataLengthCode dlc) {
	std::copy(new_data.begin(), new_data.begin() + dlc,
	          frame_data_.begin() + to_uint(FixedSizeIndex::DATA_0));

	// Zero-fill unused bytes
	std::fill(frame_data_.begin() + to_uint(FixedSizeIndex::DATA_0) + dlc,
	          frame_data_.begin() + to_uint(FixedSizeIndex::DATA_0) + 8, 0);

	frame_data_[to_uint(FixedSizeIndex::DLC)] = dlc;
	updateChecksum();

	return FrameResult<bool>::success(true);
}

FrameResult<uint8_t> impl_getData(std::size_t index) const {
	return FrameResult<uint8_t>::success(
		frame_data_[to_uint(FixedSizeIndex::DATA_0) + index]);
}

bool impl_inDataRange(std::size_t index) const {
	auto dlc = frame_data_[to_uint(FixedSizeIndex::DLC)];
	return index < dlc && index < 8;
}

// Protocol implementations
FrameResult<Type> impl_getType() const {
	return FrameResult<Type>::success(static_cast<Type>(frame_data_[to_uint(FixedSizeIndex::TYPE)]));
}

FrameResult<bool> impl_setType(Type type) {
	frame_data_[to_uint(FixedSizeIndex::TYPE)] = to_uint8(type);
	updateChecksum();
	return FrameResult<bool>::success(true);
}

FrameResult<FrameType> impl_getFrameType() const {
	return FrameResult<FrameType>::success(
		static_cast<FrameType>(frame_data_[to_uint(FixedSizeIndex::FRAME_TYPE)]));
}

FrameResult<bool> impl_setFrameType(FrameType frame_type) {
	frame_data_[to_uint(FixedSizeIndex::FRAME_TYPE)] = to_uint8(frame_type);
	updateChecksum();
	return FrameResult<bool>::success(true);
}

FrameResult<FrameFmt> impl_getFrameFmt() const {
	return FrameResult<FrameFmt>::success(
		static_cast<FrameFmt>(frame_data_[to_uint(FixedSizeIndex::FRAME_FMT)]));
}

FrameResult<bool> impl_setFrameFmt(FrameFmt frame_fmt) {
	frame_data_[to_uint(FixedSizeIndex::FRAME_FMT)] = to_uint8(frame_fmt);
	updateChecksum();
	return FrameResult<bool>::success(true);
}

FrameResult<FrameTypes::DataLengthCode> impl_getDLC() const {
	return FrameResult<FrameTypes::DataLengthCode>::success(
		frame_data_[to_uint(FixedSizeIndex::DLC)]);
}

FrameResult<bool> impl_setDLC(FrameTypes::DataLengthCode dlc) {
	frame_data_[to_uint(FixedSizeIndex::DLC)] = dlc;
	updateChecksum();
	return FrameResult<bool>::success(true);
}

FrameResult<FrameTypes::CANIdentifier> impl_getID() const {
	FrameTypes::CANIdentifier id = 0;
	id |= static_cast<uint32_t>(frame_data_[to_uint(FixedSizeIndex::ID_0)]) << 24;
	id |= static_cast<uint32_t>(frame_data_[to_uint(FixedSizeIndex::ID_1)]) << 16;
	id |= static_cast<uint32_t>(frame_data_[to_uint(FixedSizeIndex::ID_2)]) << 8;
	id |= static_cast<uint32_t>(frame_data_[to_uint(FixedSizeIndex::ID_3)]);
	return FrameResult<FrameTypes::CANIdentifier>::success(id);
}

FrameResult<bool> impl_setID(FrameTypes::CANIdentifier id) {
	frame_data_[to_uint(FixedSizeIndex::ID_0)] = (id >> 24) & 0xFF;
	frame_data_[to_uint(FixedSizeIndex::ID_1)] = (id >> 16) & 0xFF;
	frame_data_[to_uint(FixedSizeIndex::ID_2)] = (id >> 8) & 0xFF;
	frame_data_[to_uint(FixedSizeIndex::ID_3)] = id & 0xFF;
	updateChecksum();
	return FrameResult<bool>::success(true);
}

FrameResult<FrameTypes::IDArrayPair> impl_getIDArray() const {
	FrameTypes::IDArray id_array{};
	std::copy(frame_data_.begin() + to_uint(FixedSizeIndex::ID_0),
	          frame_data_.begin() + to_uint(FixedSizeIndex::ID_0) + 4,
	          id_array.begin());

	// Fixed frames always use 4 bytes for ID
	return FrameResult<FrameTypes::IDArrayPair>::success({id_array, 4});
}

// Serialization implementations
SerializationResult impl_serialize() const {
	SerializationResult result{};

	try {
		result.data = std::vector<uint8_t>(frame_data_.begin(), frame_data_.end());
		result.bytes_written = result.data.size();
		result.status = FrameOperationStatus::SUCCESS;

		// Debug info
		result.debug_info.header_bytes = 2;
		result.debug_info.id_bytes = 4;
		result.debug_info.payload_bytes = frame_data_[to_uint(FixedSizeIndex::DLC)];
		result.debug_info.checksum_bytes = 1;
		result.debug_info.calculated_checksum = calculateChecksum();

	} catch (...) {
		result.status = FrameOperationStatus::SERIALIZATION_ERROR;
	}

	return result;
}

FrameResult<bool> impl_deserialize(const std::vector<uint8_t>& data) {
	if (data.size() != traits::FRAME_SIZE) {
		return FrameResult<bool>::error(FrameOperationStatus::INVALID_FRAME_TYPE,
		                                "Invalid frame size for fixed frame");
	}

	std::copy(data.begin(), data.end(), frame_data_.begin());

	auto validation = impl_validateFrame();
	if (!validation.is_valid) {
		return FrameResult<bool>::error(FrameOperationStatus::INVALID_FRAME_TYPE,
		                                validation.description);
	}

	return FrameResult<bool>::success(true);
}

// Validation implementations
ValidationResult impl_validateFrame() const {
	// Check start byte
	if (frame_data_[to_uint(FixedSizeIndex::START)] != start_byte_) {
		return ValidationResult::invalid_start_byte();
	}

	// Check DLC
	auto dlc = frame_data_[to_uint(FixedSizeIndex::DLC)];
	if (dlc > 8) {
		return ValidationResult::invalid_dlc(to_uint(FixedSizeIndex::DLC));
	}

	// Check checksum
	if (!isValidChecksum()) {
		return ValidationResult::invalid_checksum(to_uint(FixedSizeIndex::CHECKSUM));
	}

	return ValidationResult::success();
}

ValidationResult impl_validateDataSection() const {
	auto dlc = frame_data_[to_uint(FixedSizeIndex::DLC)];
	if (dlc > 8) {
		return ValidationResult::invalid_dlc(to_uint(FixedSizeIndex::DLC));
	}
	return ValidationResult::success();
}

ValidationResult impl_validateIDSection() const {
	// For fixed frames, ID validation is always successful
	// (could add range checks for standard vs extended here)
	return ValidationResult::success();
}

private:
void updateChecksum() {
	frame_data_[to_uint(FixedSizeIndex::CHECKSUM)] = calculateChecksum();
}

FrameTypes::ChecksumValue calculateChecksum() const {
	uint8_t sum = 0;
	for (std::size_t i = to_uint(FixedSizeIndex::TYPE);
	     i <= to_uint(FixedSizeIndex::RESERVED); ++i) {
		sum += frame_data_[i];
	}
	return sum & 0xFF;
}

bool isValidChecksum() const {
	return calculateChecksum() == frame_data_[to_uint(FixedSizeIndex::CHECKSUM)];
}
};

// ================================================================================================
// VARIABLE SIZE FRAME IMPLEMENTATION (SIMPLIFIED FOR DEMONSTRATION)
// ================================================================================================

class VariableSizeFrame : public AdapterBaseFrame<VariableSizeFrame> {
private:
alignas(4) traits::StorageType frame_data_{};
std::size_t actual_size_ = 0;

public:
VariableSizeFrame() {
	frame_data_[0] = start_byte_;
	actual_size_ = 1;
}

// Implementation methods (simplified - similar pattern to FixedSizeFrame)
uint8_t& impl_subscript(std::size_t index) {
	return frame_data_[index];
}
const uint8_t& impl_subscript(std::size_t index) const {
	return frame_data_[index];
}
uint8_t& impl_at(std::size_t index) {
	if (index >= actual_size_) throw std::out_of_range("Index out of range");
	return frame_data_[index];
}
const uint8_t& impl_at(std::size_t index) const {
	if (index >= actual_size_) throw std::out_of_range("Index out of range");
	return frame_data_[index];
}

uint8_t* impl_begin() {
	return frame_data_.data();
}
uint8_t* impl_end() {
	return frame_data_.data() + actual_size_;
}
const uint8_t* impl_begin() const {
	return frame_data_.data();
}
const uint8_t* impl_end() const {
	return frame_data_.data() + actual_size_;
}

void impl_fill(uint8_t value) {
	frame_data_.fill(value);
}
std::size_t impl_size() const {
	return actual_size_;
}

// Simplified implementations for other methods...
FrameResult<FrameTypes::CANPayloadPair> impl_getDataArray() const {
	return FrameResult<FrameTypes::CANPayloadPair>::success({{}, 0});
}

FrameResult<bool> impl_setData(const FrameTypes::CANPayload&, FrameTypes::DataLengthCode) {
	return FrameResult<bool>::success(true);
}

FrameResult<uint8_t> impl_getData(std::size_t) const {
	return FrameResult<uint8_t>::success(0);
}

bool impl_inDataRange(std::size_t) const {
	return true;
}

FrameResult<Type> impl_getType() const {
	return FrameResult<Type>::success(Type::DATA_VAR);
}

FrameResult<bool> impl_setType(Type) {
	return FrameResult<bool>::success(true);
}
FrameResult<FrameType> impl_getFrameType() const {
	return FrameResult<FrameType>::success(FrameType::STD_VAR);
}
FrameResult<bool> impl_setFrameType(FrameType) {
	return FrameResult<bool>::success(true);
}
FrameResult<FrameFmt> impl_getFrameFmt() const {
	return FrameResult<FrameFmt>::success(FrameFmt::DATA_VAR);
}
FrameResult<bool> impl_setFrameFmt(FrameFmt) {
	return FrameResult<bool>::success(true);
}
FrameResult<FrameTypes::DataLengthCode> impl_getDLC() const {
	return FrameResult<FrameTypes::DataLengthCode>::success(0);
}
FrameResult<bool> impl_setDLC(FrameTypes::DataLengthCode) {
	return FrameResult<bool>::success(true);
}
FrameResult<FrameTypes::CANIdentifier> impl_getID() const {
	return FrameResult<FrameTypes::CANIdentifier>::success(0);
}
FrameResult<bool> impl_setID(FrameTypes::CANIdentifier) {
	return FrameResult<bool>::success(true);
}
FrameResult<FrameTypes::IDArrayPair> impl_getIDArray() const {
	return FrameResult<FrameTypes::IDArrayPair>::success({{}, 0});
}

SerializationResult impl_serialize() const {
	SerializationResult result{};
	result.data = std::vector<uint8_t>(frame_data_.begin(), frame_data_.begin() + actual_size_);
	result.status = FrameOperationStatus::SUCCESS;
	result.bytes_written = actual_size_;
	return result;
}

FrameResult<bool> impl_deserialize(const std::vector<uint8_t>& data) {
	if (data.size() > traits::MAX_FRAME_SIZE) {
		return FrameResult<bool>::error(FrameOperationStatus::BUFFER_OVERFLOW, "Frame too large");
	}
	std::copy(data.begin(), data.end(), frame_data_.begin());
	actual_size_ = data.size();
	return FrameResult<bool>::success(true);
}

ValidationResult impl_validateFrame() const {
	return ValidationResult::success();
}
ValidationResult impl_validateDataSection() const {
	return ValidationResult::success();
}
ValidationResult impl_validateIDSection() const {
	return ValidationResult::success();
}
};

// ================================================================================================
// STREAMING OPERATORS FOR DEBUGGING
// ================================================================================================

std::ostream& operator<<(std::ostream& os, FrameOperationStatus status) {
	switch (status) {
	case FrameOperationStatus::SUCCESS: return os << "SUCCESS";
	case FrameOperationStatus::INVALID_DLC: return os << "INVALID_DLC";
	case FrameOperationStatus::INVALID_ID: return os << "INVALID_ID";
	case FrameOperationStatus::INVALID_CHECKSUM: return os << "INVALID_CHECKSUM";
	case FrameOperationStatus::BUFFER_OVERFLOW: return os << "BUFFER_OVERFLOW";
	case FrameOperationStatus::INVALID_FRAME_TYPE: return os << "INVALID_FRAME_TYPE";
	case FrameOperationStatus::SERIALIZATION_ERROR: return os << "SERIALIZATION_ERROR";
	default: return os << "UNKNOWN(" << static_cast<int>(status) << ")";
	}
}

std::ostream& operator<<(std::ostream& os, const ValidationResult& result) {
	os << "ValidationResult{valid=" << result.is_valid
	   << ", error=" << static_cast<int>(result.error_code)
	   << ", byte_index=" << result.error_byte_index
	   << ", description=\"" << result.description << "\"}";
	return os;
}

// ================================================================================================
// USAGE EXAMPLES AND TEMPLATE FUNCTIONS
// ================================================================================================

// Template function that works with any frame type
template<typename FrameType>
void processFrame(AdapterBaseFrame<FrameType>& frame) {
	using traits = typename FrameType::traits;

	std::cout << "\n=== Processing " << (traits::IS_VARIABLE_SIZE ? "Variable" : "Fixed")
	          << " Size Frame ===" << std::endl;

	// Validation
	auto validation = frame.validateFrame();
	std::cout << "Validation: " << validation << std::endl;

	if (validation.is_valid) {
		// Get frame information
		auto id_result = frame.getID();
		if (id_result.is_success()) {
			std::cout << "Frame ID: 0x" << std::hex << id_result.value << std::dec
			          << " (status: " << id_result.status << ")" << std::endl;
		}

		auto data_result = frame.getDataArray();
		if (data_result.is_success()) {
			std::cout << "Frame data (DLC=" << static_cast<int>(data_result.value.second) << "): ";
			for (std::size_t i = 0; i < data_result.value.second; ++i) {
				std::cout << "0x" << std::hex << static_cast<int>(data_result.value.first[i]) << " ";
			}
			std::cout << std::dec << std::endl;
		}

		// Serialization
		auto serial_result = frame.serialize();
		if (serial_result.is_success()) {
			std::cout << "Serialized " << serial_result.bytes_written << " bytes in "
			          << serial_result.serialization_time.count() << " microseconds" << std::endl;
		}

		// Debug info
		auto debug_info = frame.getDebugInfo();
		std::cout << "Operations performed: " << debug_info.operation_count << std::endl;
	}
}

} // namespace USBCANBridge_CRTP

// ================================================================================================
// DEMONSTRATION MAIN FUNCTION
// ================================================================================================

// Uncomment to test the implementation
/*
   int main() {
    using namespace USBCANBridge_CRTP;

    std::cout << "=== CRTP USB-CAN Frame Implementation Demo ===" << std::endl;

    // Create frame instances
    FixedSizeFrame fixed_frame;
    VariableSizeFrame variable_frame;

    // Configure fixed frame
    fixed_frame.setID(0x123);
    fixed_frame.setDLC(4);

    FrameTypes::CANPayload payload = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x00, 0x00, 0x00};
    fixed_frame.setData(payload, 4);

    // Process both frame types with the same template function
    processFrame(fixed_frame);
    processFrame(variable_frame);

    // Demonstrate compile-time polymorphism
    std::cout << "\n=== Compile-time Features ===" << std::endl;
    std::cout << "Fixed frame max size: " << fixed_frame.max_frame_size() << std::endl;
    std::cout << "Variable frame max size: " << variable_frame.max_frame_size() << std::endl;
    std::cout << "Fixed frame is variable: " << fixed_frame.isVariableSizeProtocol() << std::endl;
    std::cout << "Variable frame is variable: " << variable_frame.isVariableSizeProtocol() << std::endl;

    return 0;
   }
 */

/** * USAGE EXAMPLES (FOR DOCUMENTATION)
 * // Both types use the same interface but generate different optimized code
   FixedSizeFrame fixed;
   VariableSizeFrame variable;

   // Template function works with both - generates specialized versions
   processFrame(fixed);    // Optimized for fixed frames
   processFrame(variable); // Optimized for variable frames

   // Smart returns with error handling
   auto result = fixed.getID();
   if (result.is_success()) {
   std::cout << "ID: 0x" << std::hex << result.value << std::endl;
   } else {
   std::cout << "Error: " << result.debug_message << std::endl;
   }
 */