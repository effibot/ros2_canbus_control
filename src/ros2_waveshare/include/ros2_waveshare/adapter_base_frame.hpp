/**
 * @file adapter_base_frame.hpp
 * @brief Abstract base class for all USB-CAN adapter frames
 * This file defines the abstract base class that provides the common interface
 * and basic functionality for all frame types in the USB-CAN adapter implementation.
 *
 * @author Andrea Efficace
 * @date September 2025
 */

#pragma once

#include "usb_can_common.hpp"

#include <vector>
#include <cstdint>
#include <memory>

namespace USBCANBridge
{

/**
 * @brief Abstract base class for all USB-CAN adapter frames
 * This class provides the common interface and basic functionality for all frame types.
 * It encapsulates the start byte and defines pure virtual methods that derived classes
 * must implement for specific frame handling.
 */
class AdapterBaseFrame {
protected:
const uint8_t start_byte_;

public:
// Constructor
explicit AdapterBaseFrame() : start_byte_(to_uint8(Constants::START_BYTE)) {
}

// Virtual destructor for proper cleanup
virtual ~AdapterBaseFrame() = default;

// Disable copy constructor and copy assignment operator to prevent slicing
AdapterBaseFrame(const AdapterBaseFrame&) = delete;
AdapterBaseFrame& operator=(const AdapterBaseFrame&) = delete;

// Enable move semantics
AdapterBaseFrame(AdapterBaseFrame&&) = default;
AdapterBaseFrame& operator=(AdapterBaseFrame&&) = default;

// Basic validation
bool isValidStart() const {
	return start_byte_ == to_uint8(Constants::START_BYTE);
}

// Pure virtual interface - must be implemented by derived classes
virtual uint8_t& operator[](std::size_t index) = 0;
virtual const uint8_t& operator[](std::size_t index) const = 0;
virtual uint8_t& at(std::size_t index) = 0;
virtual const uint8_t& at(std::size_t index) const = 0;

// STL-style interface
virtual uint8_t* begin() = 0;
virtual uint8_t* end() = 0;
virtual const uint8_t* begin() const = 0;
virtual const uint8_t* end() const = 0;
virtual void fill(uint8_t value) = 0;
virtual std::size_t size() const = 0;

// Data manipulation interface
virtual void setData(const std::vector<uint8_t>& new_data) = 0;
virtual void setData(size_t index, uint8_t value) = 0;
virtual std::vector<uint8_t> getData() const = 0;
virtual uint8_t getData(size_t index) const = 0;
virtual bool inDataRange(size_t index) const = 0;

// Protocol handling interface
virtual void setType(Type type) = 0;
virtual Type getType() const = 0;
virtual void setFrameType(FrameType frame_type) = 0;
virtual FrameType getFrameType() const = 0;
virtual void setFrameFmt(FrameFmt frame_fmt) = 0;
virtual FrameFmt getFrameFmt() const = 0;
virtual void setDLC(uint8_t dlc) = 0;
virtual uint8_t getDLC() const = 0;
virtual bool isValidLength() const = 0;
virtual void setID(uint32_t id) = 0;
virtual uint32_t getID() const = 0;
virtual std::vector<uint8_t> getIDBytes() const = 0;
virtual void setIDBytes(const std::vector<uint8_t>& id_bytes) = 0;
virtual bool isValidID() const = 0;

// Serialization interface
virtual std::vector<uint8_t> serialize() const = 0;
virtual bool deserialize(const std::vector<uint8_t>& data) = 0;
virtual bool isValidFrame() const = 0;
};

} // namespace USBCANBridge
