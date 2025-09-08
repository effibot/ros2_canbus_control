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
/**
 * @brief Start byte marker present at the beginning of every adapter frame.
 *
 * This constant is initialized from the protocol constant START_BYTE and is
 * used by all derived classes to validate frame integrity. It is immutable
 * after construction.
 */
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
/**
 * @name Random access interface
 * @brief Provide array-like access to the underlying frame storage.
 *
 * These operators allow treating the frame as a contiguous byte array.
 * Implementations must throw std::out_of_range (for at()) when an invalid
 * index is requested. operator[] has undefined behavior for invalid indices.
 * @{ */
virtual uint8_t& operator[](std::size_t index) = 0;
virtual const uint8_t& operator[](std::size_t index) const = 0;
virtual uint8_t& at(std::size_t index) = 0;
virtual const uint8_t& at(std::size_t index) const = 0;
/** @} */

// STL-style interface
/**
 * @name STL-style iteration
 * @brief Expose iterators over the raw underlying bytes of the frame.
 *
 * begin() points to index 0 (start byte) and end() points one past the last
 * valid byte (i.e., &data[size()]). Implementations must guarantee that the
 * memory layout is contiguous over [begin(), end()).
 * @{ */
virtual uint8_t* begin() = 0;
virtual uint8_t* end() = 0;
virtual const uint8_t* begin() const = 0;
virtual const uint8_t* end() const = 0;
virtual void fill(uint8_t value) = 0;
virtual std::size_t size() const = 0;
/** @} */

// Data manipulation interface
/**
 * @name Data field accessors
 * @brief Access and modify only the CAN payload portion of the frame.
 *
 * Implementations should enforce maximum payload length (<= 8 bytes) and
 * ensure DLC consistency where applicable.
 * @{ */
virtual void setData(const std::vector<uint8_t>& new_data) = 0;
virtual void setData(size_t index, uint8_t value) = 0;
virtual std::vector<uint8_t> getData() const = 0;
virtual uint8_t getData(size_t index) const = 0;
virtual bool inDataRange(size_t index) const = 0;
/** @} */

// Protocol handling interface
/**
 * @name Protocol meta-data accessors
 * @brief Manage protocol specific attributes (type, frame type, format, DLC, ID).
 *
 * These functions must validate arguments and throw std::invalid_argument in
 * case of invalid protocol values.
 * @{ */
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
/** @} */

// Serialization interface
/**
 * @name Serialization
 * @brief Convert between in-memory representation and raw byte vectors.
 *
 * serialize() must return a vector sized exactly to the frame size (fixed or
 * variable). deserialize() must parse and populate the object, returning true
 * when the resulting frame is valid. isValidFrame() performs full structural
 * and semantic validation (start byte, length, DLC, ID range, checksums, etc.).
 * @{ */
virtual std::vector<uint8_t> serialize() const = 0;
virtual bool deserialize(const std::vector<uint8_t>& data) = 0;
virtual bool isValidFrame() const = 0;
/** @} */
};

} // namespace USBCANBridge
