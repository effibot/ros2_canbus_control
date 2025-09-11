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
#include "usb_can_common.hpp"

namespace USBCANbridge {
// * template alias for error codes
template <typename T>
struct Result {
	T value;
	Status status;
	const char* message;
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
// Protected constructor to prevent direct instantiation
BaseFrame() = default;
// start byte for frame synchronization
static constexpr std::uint8_t START_BYTE = Constants::START_BYTE;
public:
explicit BaseFrame() : BaseFrame() {
}
// Non-virtual interface methods that delegate to derived class
uint8_t& operator[](std::size_t index) {
	return derived().operator[](index);
}
const uint8_t& operator[](std::size_t index) const {
	return derived().operator[](index);
}

// ... other interface methods
};

template<typename Frame>
struct FrameTraits;

template<>
struct FrameTraits<FixedSizeFrame> {
	static constexpr std::size_t FRAME_SIZE = 20;
	static constexpr std::size_t ID_SIZE = 4;
	static constexpr std::size_t DATA_SIZE = 8;
	static constexpr bool IS_VARIABLE_SIZE = false;
	using StorageType = std::array<uint8_t, FRAME_SIZE>;
};

template<>
struct FrameTraits<VariableSizeFrame> {
	static constexpr std::size_t MAX_FRAME_SIZE = 13; // 1+1+4+8-1
	static constexpr std::size_t MAX_ID_SIZE = 4;
	static constexpr std::size_t MAX_DATA_SIZE = 8;
	static constexpr bool IS_VARIABLE_SIZE = true;
	using StorageType = std::array<uint8_t, MAX_FRAME_SIZE>;
};

} // namespace USBCANbridge