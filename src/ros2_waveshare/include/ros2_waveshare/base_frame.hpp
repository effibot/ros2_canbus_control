/**
 * @file base_frame.hpp
 * @author Andrea Efficace (andrea.efficace1@gmail.com)
 * @brief Base class for Waveshare USB-CAN-A adapter frame handling.
 * This implements the CRTP pattern.
 * @version 0.1
 * @date 2025-09-11
 */

#pragma once
#include <array>
#include <cstdint>
#include <cstddef>
#include <vector>
#include "common.hpp"
#include "error.hpp"

namespace USBCANBridge {
// * Smart return type with error code
template <typename T>
struct Result {
	T value;
	std::error_code status;

	// Convenience methods
	bool ok() const {
		return status == Status::SUCCESS;
	}
	bool fail() const {
		return status != Status::SUCCESS;
	}
	explicit operator bool() const {
		return ok();
	}
	bool operator!() const {
		return !ok();
	}
	static std::string to_string() {
		return status.message();
	}
	static Result success(T val) {
		return {std::move(val), Status::SUCCESS};
	}
	static Result error(Status err) {
		return {T{}, err};
	}
};

// * Frame Traits declaration for compile-time type selection
// Forward declarations
class FixedSizeFrame;
class VariableSizeFrame;

template<typename Frame>
struct FrameTraits;
/**
 *! FrameTraits specializations are defined in fixed_frame.hpp and var_frame.hpp
 *! the needed types are: FRAME_SIZE, ID_SIZE, DATA_SIZE, StorageType, IDType, DataType, PayloadPair, IDPair
 */


// * templating to allow static polymorphism via CRTP
template <typename Derived>
class BaseFrame {
private:
// Access to the concrete Derived class
Derived& derived() {
	return static_cast<Derived&>(*this);
}
const Derived& derived() const {
	return static_cast<const Derived&>(*this);
}
protected:
// start byte for frame synchronization
static constexpr std::uint8_t START_BYTE = Constants::START_BYTE;

public:
// Constructors
explicit BaseFrame() {
};

// Bring traits into scope and define useful types
using traits = FrameTraits<Derived>;
using storage = typename traits::StorageType;
using payload = typename traits::PayloadPair;
using id_t = typename traits::IDPair;
// Disable copy/assignment to avoid slicing
BaseFrame(const BaseFrame&) = delete;
BaseFrame& operator=(const BaseFrame&) = delete;

// Enable move semantics
BaseFrame(BaseFrame&&) = default;
BaseFrame& operator=(BaseFrame&&) = default;
virtual ~BaseFrame() = default;


// * Public interface methods calling the implementation in Derived class
// Index access operators
uint8_t& operator[](std::size_t index) {
	return derived().impl_subscript(index);
};
const uint8_t& operator[](std::size_t index) const {
	return derived().impl_subscript(index);
};
uint8_t& at(std::size_t index) {
	return derived().impl_at(index);
};
const uint8_t& at(std::size_t index) const {
	return derived().impl_at(index);
};
// STL-style interface
uint8_t* begin() {
	return derived().impl_begin();
};
uint8_t* end() {
	return derived().impl_end();
};
const uint8_t* begin() const {
	return derived().impl_begin();
};
const uint8_t* end() const {
	return derived().impl_end();
};
void fill(uint8_t value) {
	derived().impl_fill(value);
};
std::size_t size() const {
	return derived().impl_size();
};

// * Interface for data manipulation
Result<payload> getData() const {
	return derived().impl_getData();
};
Result<payload> getData(size_t index) const {
	return derived().impl_getData(index);
};
Result<bool> setData(const payload& new_data){
	// * invoke the derived validateData method
	auto validation = validateData(new_data);
	if (validation.fail()) {
		return Result<bool>::error(validation.status);
	}
	// * if validation passed, call the implementation
	return derived().impl_setData(new_data);
};
// * Interface for type manipulation
Result<Type> getType() const {
	return derived().impl_getType();
};
Result<bool> setType(Type type) {
	// * invoke the derived validateType method
	auto validation = validateType(type);
	if (validation.fail()) {
		return Result<bool>::error(validation.status);
	}
	// * if validation passed, call the implementation
	return derived().impl_setType(type);
};
// * Interface for frame type manipulation
Result<FrameType> getFrameType() const {
	return derived().impl_getFrameType();
};
Result<bool> setFrameType(FrameType frame_type) {
	// * invoke the derived validateFrameType method
	auto validation = validateFrameType(frame_type);
	if (validation.fail()) {
		return Result<bool>::error(validation.status);
	}
	// * if validation passed, call the implementation
	return derived().impl_setFrameType(frame_type);
};
// * Interface for frame format manipulation
Result<FrameFmt> getFrameFmt() const {
	return derived().impl_getFrameFmt();
};
Result<bool> setFrameFmt(FrameFmt frame_fmt) {
	// * invoke the derived validateFrameFmt method
	auto validation = validateFrameFmt(frame_fmt);
	if (validation.fail()) {
		return Result<bool>::error(validation.status);
	}
	// * if validation passed, call the implementation
	return derived().impl_setFrameFmt(frame_fmt);
};
// * Interface for DLC manipulation
Result<uint8_t> getDLC() const {
	return derived().impl_getDLC();
};
Result<bool> setDLC(uint8_t dlc) {
	// * invoke the derived validateDLC method
	auto validation = validateDLC(dlc);
	if (validation.fail()) {
		return Result<bool>::error(validation.status);
	}
	// * if validation passed, call the implementation
	return derived().impl_setDLC(dlc);
};
//* Interface for ID manipulation
Result<id_t> getID() const {
	return derived().impl_getID();
};
Result<bool> setID(const id_t& id) {
	// * invoke the derived validateID method
	auto validation = validateID(id);
	if (validation.fail()) {
		return Result<bool>::error(validation.status);
	}
	// * if validation passed, call the implementation
	return derived().impl_setID(id);
};
// * Interface for serialization/deserialization
Result<storage> serialize() const {
	// * Invoke the validateFrame method
	auto validation = validateFrame();
	if (validation.fail()) {
		return Result<storage>::error(validation.status);
	}
	// * If validation passed, call the implementation
	return derived().impl_serialize();
};
Result<bool> deserialize(const std::vector<uint8_t>& data) {
	// * invoke the derived validateFrame method
	auto validation = validateFrameData(data);
	if (validation.fail()) {
		return Result<bool>::error(validation.status);
	}
	// * if validation passed, call the implementation
	return derived().impl_deserialize(data);
};
// * Interface for frame validation
Result<bool> validateFrame() const {
	return derived().impl_validateFrame();
};
Result<bool> validateData() const {
	return derived().impl_validateData();
};
Result<bool> validateID() const {
	return derived().impl_validateID();
};
Result<bool> validateType(Type type) const {
	return derived().impl_validateType(type);
};
Result<bool> validateFrameType(FrameType frame_type) const {
	return derived().impl_validateFrameType(frame_type);
};
Result<bool> validateFrameFmt(FrameFmt frame_fmt) const {
	return derived().impl_validateFrameFmt(frame_fmt);
};
Result<bool> validateDLC(uint8_t dlc) const {
	return derived().impl_validateDLC(dlc);
};
Result<bool> validateID(const id_t& id) const {
	return derived().impl_validateID(id);
};
};
} // namespace USBCANBridge
