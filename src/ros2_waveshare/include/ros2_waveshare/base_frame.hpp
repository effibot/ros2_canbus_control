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

// * Fixed size frame traits
template<>
struct FrameTraits<FixedSizeFrame> {
	static constexpr std::size_t FRAME_SIZE = 20;
	static constexpr std::size_t ID_SIZE = 4;
	static constexpr std::size_t DATA_SIZE = 8;


	using StorageType = std::array<uint8_t, FRAME_SIZE>;
	using IDType = std::array<uint8_t, ID_SIZE>;
	using DataType = std::array<uint8_t, DATA_SIZE>;
	using SerializeReturnType = std::array<uint8_t, FRAME_SIZE>;
};

// * Variable size frame traits
template<>
struct FrameTraits<VariableSizeFrame> {
	static constexpr std::size_t MAX_FRAME_SIZE = 13; // 1+1+4+8-1
	static constexpr std::size_t MAX_ID_SIZE = 4;
	static constexpr std::size_t MAX_DATA_SIZE = 8;

	using StorageType = std::array<uint8_t, MAX_FRAME_SIZE>;
	using IDType = std::array<uint8_t, MAX_ID_SIZE>;
	using DataType = std::array<uint8_t, MAX_DATA_SIZE>;
	using SerializeReturnType = std::vector<uint8_t>;
};

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
}

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


} // namespace USBCANBridge