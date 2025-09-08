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

#include <vector>
#include <cstdint>

namespace USBCANBridge
{
#pragma pack(push, 1)

/**
 * @brief Variable length frame class
 * This class represents the variable length frame format used by the USB-CAN-A adapter.
 * It provides dynamic sizing based on the frame type and data length.
 */
class VariableSizeFrame : public AdapterBaseFrame {
private:
    /** @brief Constant type header (always Type::DATA_VAR). */
    const uint8_t type_header_;
    /** @brief Frame type (STD_VAR or EXT_VAR) determines ID size. */
    uint8_t frame_type_;
    /** @brief Frame format (DATA_VAR or REMOTE_VAR). */
    uint8_t frame_fmt_;
    /** @brief Data Length Code (0..8, equals data_.size()). */
    uint8_t dlc_;
    /** @brief CAN identifier bytes (2 for standard, 4 for extended). */
    std::vector<uint8_t> id_;
    /** @brief CAN payload sized dynamically according to dlc_. */
    std::vector<uint8_t> data_;
    /** @brief End marker byte (Constants::END_BYTE). */
    const uint8_t end_byte_;

    // Utility indices for frame structure
    VarSizeIndex ID_END_;
    VarSizeIndex DATA_START_;
    VarSizeIndex DATA_END_;
    VarSizeIndex FRAME_END_;

    void updateIndices();
    void validateIndex(std::size_t index) const;

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
    std::vector<uint8_t> getData() const override;
    uint8_t getData(size_t index) const override;

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
    std::vector<uint8_t> getIDBytes() const override;
    void setIDBytes(const std::vector<uint8_t>& id_bytes) override;
    bool isValidID() const override;

    // Validation and serialization
    bool isValidFrame() const override;
    std::vector<uint8_t> serialize() const override;
    bool deserialize(const std::vector<uint8_t>& raw_data) override;
};

#pragma pack(pop)

} // namespace USBCANBridge
