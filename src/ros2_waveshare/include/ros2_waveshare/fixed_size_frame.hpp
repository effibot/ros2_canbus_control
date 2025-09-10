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
#include <vector>
#include <cstdint>

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
/** @brief Protocol message header byte (constant, typically 0x55). */
const uint8_t msg_header_;
/** @brief Frame meta type (DATA_FIXED or CONF_FIXED). */
uint8_t type_;
/** @brief CAN identifier type (standard or extended fixed). */
uint8_t frame_type_;
/** @brief Frame format (data or remote request). */
uint8_t frame_fmt_;
/** @brief Raw CAN identifier bytes (little-endian internal ordering). */
std::array<uint8_t, 4> id_bytes_;
/** @brief Data Length Code (0..8). */
uint8_t dlc_;
/** @brief CAN payload (always 8 bytes allocated; only first dlc_ are valid). */
std::array<uint8_t, 8> data_;
/** @brief Reserved protocol byte (constant, usually 0x00). */
const uint8_t reserved_;
/** @brief Low 8 bits checksum over bytes type..reserved (inclusive). */
uint8_t checksum_;

// Private helper methods
uint8_t calculateChecksum() const;
void validateIndex(std::size_t index) const;

public:
// Constructors
FixedSizeFrame();
explicit FixedSizeFrame(Type type, FrameType frame_type, FrameFmt fmt);

// Destructor
~FixedSizeFrame() override = default;

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

// Data access methods
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

// ID access methods
std::pair<std::array<uint8_t, 4>, uint8_t> getIDArray() const override;

// Checksum methods
uint8_t getChecksum() const;
void updateChecksum();

// Validation and serialization
bool isValidFrame() const override;
std::vector<uint8_t> serialize() const override;
bool deserialize(const std::vector<uint8_t>& raw_data) override;
};

#pragma pack(pop)

} // namespace USBCANBridge
