/**
 * @file variable_size_frame.hpp
 * @brief Variable length frame class for USB-CAN adapter
 * This file defines the VariableSizeFrame class that represents the variable length
 * frame format used by the USB-CAN-A adapter. It provides dynamic sizing based on
 * the frame type and data length.
 *
 * @author Andrea Efficace
 * @date September 2025
 */

#pragma once

#include "adapter_base_frame.hpp"
#include "usb_can_common.hpp"

#include <array>
#include <vector>  // Still needed for setData() input parameter
#include <cstdint>
#include <cstdlib>  // for std::abort

namespace USBCANBridge
{
#pragma pack(push, 1)

/**
 * @brief Variable length frame class
 * This class represents the variable length frame format used by the USB-CAN-A adapter.
 * It provides dynamic sizing based on the frame type and data length, using stack-allocated
 * arrays for better memory efficiency and cache performance.
 */
class VariableSizeFrame : public AdapterBaseFrame {
private:
/** @brief Constant type header (always Type::DATA_VAR). */
const uint8_t type_header_;
/** @brief Frame type (STD_VAR or EXT_VAR) determines ID size. */
uint8_t frame_type_;
/** @brief Frame format (DATA_VAR or REMOTE_VAR). */
uint8_t frame_fmt_;
/** @brief Data Length Code (0..8, equals actual data size). */
uint8_t dlc_;
/** @brief End marker byte (Constants::END_BYTE). */
const uint8_t end_byte_;

// Internal storage using fixed-size arrays for better performance
static constexpr std::size_t MAX_ID_SIZE = 4;       // Extended CAN ID
static constexpr std::size_t MAX_DATA_SIZE = 8;     // CAN data payload

/** @brief ID storage (max 4 bytes, actual size in id_size_). */
alignas(4) std::array<uint8_t, MAX_ID_SIZE> id_{};
/** @brief Data storage (max 8 bytes, actual size in dlc_). */
alignas(8) std::array<uint8_t, MAX_DATA_SIZE> data_{};
/** @brief Actual ID size (2 for standard, 4 for extended). */
uint8_t id_size_{2};

// Index calculation helpers for frame layout
constexpr std::size_t getDataStart() const noexcept {
	return 5 + id_size_; // start(1) + type(1) + frame_type(1) + frame_fmt(1) + dlc(1) + id_bytes
}

constexpr std::size_t getFrameEnd() const noexcept {
	return getDataStart() + dlc_;
}

// Bounds checking with different behavior for debug vs release builds
inline void validateIndex(std::size_t index) const {
	#ifndef NDEBUG
	if (index > getFrameEnd()) {
		throw std::out_of_range("Index " + std::to_string(index) +
		                        " out of range for VariableSizeFrame (max: " +
		                        std::to_string(getFrameEnd()) + ")");
	}
	#else
	// In release builds, prevent undefined behavior with immediate termination
	if (__builtin_expect(index > getFrameEnd(), 0)) {
		std::abort();
	}
	#endif
}

public:
// Constructors
VariableSizeFrame();
explicit VariableSizeFrame(FrameType frame_type, FrameFmt frame_fmt, uint8_t dlc);

// Destructor
~VariableSizeFrame() override = default;

// Index access operators
uint8_t& operator[](std::size_t index) override;
const uint8_t& operator[](std::size_t index) const override;
uint8_t& at(std::size_t index) override;
const uint8_t& at(std::size_t index) const override;

// STL-style interface
uint8_t* begin() override;
uint8_t* end() override;
const uint8_t* begin() const override;
const uint8_t* end() const override;
void fill(uint8_t value) override;
std::size_t size() const override;

// Data range validation
bool inDataRange(size_t index) const override;

// Data manipulation methods
void setData(const std::vector<uint8_t>& new_data) override;
void setData(size_t index, uint8_t value) override;
uint8_t getData(size_t index) const override;

// Array-based data access returning both array and valid size
std::pair<std::array<uint8_t, 8>, uint8_t> getDataArray() const override;

// Protocol handling methods
void setType(Type type) override;
Type getType() const override;
void setFrameType(FrameType frame_type) override;
FrameType getFrameType() const override;
void setFrameFmt(FrameFmt frame_fmt) override;
FrameFmt getFrameFmt() const override;
void setDLC(uint8_t dlc) override;
uint8_t getDLC() const override;
bool isValidLength() const override;
void setID(uint32_t id_value) override;
uint32_t getID() const override;
void setIDBytes(const std::vector<uint8_t>& id_bytes) override;
bool isValidID() const override;

// Array-based ID access returning both array and valid size
std::pair<std::array<uint8_t, 4>, uint8_t> getIDArray() const override;

// Validation and serialization
bool isValidFrame() const override;
std::vector<uint8_t> serialize() const override;
bool deserialize(const std::vector<uint8_t>& raw_data) override;
};

#pragma pack(pop)

} // namespace USBCANBridge
