/**
 * @file usb_can_frame.hpp
 * @brief USB-CAN adapter frame handling for ROS2
 */

#pragma once

#include "usb_can_common.hpp"
#include <vector>
#include <cstdint>
#include <memory>
#include <array>
#include <algorithm>
#include <cstring>
#include <numeric>

namespace usb_can_bridge
{

// Frame type enumeration
enum class USBCANFrameType {
	STANDARD_VARIABLE,
	EXTENDED_VARIABLE,
	STANDARD_FIXED_20,
	EXTENDED_FIXED_20,
	SETTING_FRAME
};

// Base frame structure
#pragma pack(push, 1)
struct USBCANAdapterBaseFrame {
	const uint8_t start_byte = static_cast<uint8_t>(USBCANConst::START_BYTE);
	uint8_t frame_info;

	USBCANAdapterBaseFrame() : frame_info(0) {
	}
	explicit USBCANAdapterBaseFrame(uint8_t info) : frame_info(info) {
	}

	bool isValidStart() const {
		return start_byte == static_cast<uint8_t>(USBCANConst::START_BYTE);
	}
};

// 20-byte frame using std::array for optimal performance
struct USBCANAdapter20ByteFrame : public USBCANAdapterBaseFrame {
	std::array<uint8_t, 17> data; // Fixed 17 bytes - better than raw array
	uint8_t checksum;

	USBCANAdapter20ByteFrame() : USBCANAdapterBaseFrame() {
		data.fill(0);
		checksum = 0;
	}

	explicit USBCANAdapter20ByteFrame(uint8_t info) : USBCANAdapterBaseFrame(info) {
		data.fill(0);
		checksum = 0;
	}

	// STL-like interface while maintaining performance
	uint8_t& operator[](size_t index) {
		return data[index];
	}
	const uint8_t& operator[](size_t index) const {
		return data[index];
	}

	uint8_t& at(size_t index) {
		return data.at(index);
	}
	const uint8_t& at(size_t index) const {
		return data.at(index);
	}

	size_t size() const {
		return data.size();
	}

	auto begin() {
		return data.begin();
	}
	auto end() {
		return data.end();
	}
	auto begin() const {
		return data.begin();
	}
	auto end() const {
		return data.end();
	}

	// Vector-like operations
	void fill(uint8_t value) {
		data.fill(value);
	}

	// Safe data access
	void setData(const std::vector<uint8_t>& input_data) {
		size_t copy_size = std::min(input_data.size(), data.size());
		std::copy(input_data.begin(), input_data.begin() + copy_size, data.begin());
		// Fill remaining with zeros if input is smaller
		if (copy_size < data.size()) {
			std::fill(data.begin() + copy_size, data.end(), 0);
		}
	}

	std::vector<uint8_t> getData() const {
		return std::vector<uint8_t>(data.begin(), data.end());
	}
};

// Variable length frame using std::vector for flexibility
struct USBCANAdapterVariableFrame : public USBCANAdapterBaseFrame {

	std::vector<uint8_t> id_bytes;
	std::vector<uint8_t> data_bytes;
	uint8_t end_byte;

	USBCANAdapterVariableFrame() : USBCANAdapterBaseFrame(),
		end_byte(static_cast<uint8_t>(USBCANConst::MSG_HEADER)) {
	}

	explicit USBCANAdapterVariableFrame(uint8_t info) : USBCANAdapterBaseFrame(info),
		end_byte(static_cast<uint8_t>(USBCANConst::MSG_HEADER)) {
	}

	// Safe data manipulation
	void setIDBytes(const std::vector<uint8_t>& id) {
		id_bytes = id;
	}

	void setDataBytes(const std::vector<uint8_t>& data) {
		data_bytes = data;
		if (data_bytes.size() > 8) {
			data_bytes.resize(8); // CAN data max 8 bytes
		}
	}

	void reserveCapacity(size_t id_size, size_t data_size) {
		id_bytes.reserve(id_size);
		data_bytes.reserve(data_size);
	}
};
#pragma pack(pop)

// Abstract base class for all frames
class USBCANFrame {
protected:
USBCANAdapterBaseFrame* base_frame_ptr;

public:
USBCANFrame() : base_frame_ptr(nullptr) {
}
virtual ~USBCANFrame() = default;

virtual std::vector<uint8_t> serialize() const = 0;
virtual bool deserialize(const std::vector<uint8_t>& data) = 0;
virtual USBCANFrameType getType() const = 0;
virtual size_t getFrameSize() const = 0;
virtual bool isValid() const = 0;

bool hasValidStart() const {
	return base_frame_ptr && base_frame_ptr->isValidStart();
}

uint8_t getFrameInfo() const {
	return base_frame_ptr ? base_frame_ptr->frame_info : 0;
}

protected:
uint8_t calculateChecksum20Byte(const USBCANAdapter20ByteFrame& frame) const {
	uint32_t checksum = 0;
	checksum += frame.frame_info;

	// Use STL algorithm for clarity and potential optimization
	checksum += std::accumulate(frame.data.begin(), frame.data.end(), 0U);

	return static_cast<uint8_t>(checksum & 0xFF);
}

bool validateChecksum20Byte(const USBCANAdapter20ByteFrame& frame) const {
	return frame.checksum == calculateChecksum20Byte(frame);
}
};

// Base class for 20-byte frames using std::array
class USBCANFrame20Byte : public USBCANFrame {
protected:
USBCANAdapter20ByteFrame frame_data;

public:
USBCANFrame20Byte() : USBCANFrame() {
	base_frame_ptr = &frame_data;
}

explicit USBCANFrame20Byte(uint8_t frame_info) : USBCANFrame(), frame_data(frame_info) {
	base_frame_ptr = &frame_data;
}

size_t getFrameSize() const override {
	return 20;
}

std::vector<uint8_t> serialize() const override {
	// Calculate checksum before serializing
	const_cast<USBCANFrame20Byte*>(this)->frame_data.checksum =
		calculateChecksum20Byte(frame_data);

	std::vector<uint8_t> result;
	result.reserve(20);

	result.push_back(frame_data.start_byte);
	result.push_back(frame_data.frame_info);

	// Use STL algorithms for better performance and clarity
	result.insert(result.end(), frame_data.data.begin(), frame_data.data.end());
	result.push_back(frame_data.checksum);

	return result;
}

bool deserialize(const std::vector<uint8_t>& data) override {
	if (data.size() != 20) return false;

	frame_data.frame_info = data[1];

	// Copy data using STL algorithm
	std::copy(data.begin() + 2, data.begin() + 19, frame_data.data.begin());
	frame_data.checksum = data[19];

	return validateChecksum20Byte(frame_data);
}

bool isValid() const override {
	return hasValidStart() && validateChecksum20Byte(frame_data);
}

// Provide safe data access methods
void setDataBytes(const std::vector<uint8_t>& data) {
	frame_data.setData(data);
}

std::vector<uint8_t> getDataBytes() const {
	return frame_data.getData();
}

// Safe indexed access
uint8_t getDataByte(size_t index) const {
	return frame_data.at(index);
}

void setDataByte(size_t index, uint8_t value) {
	frame_data.at(index) = value;
}
};

// Settings frame with enhanced safety
class USBCANSettingsFrame : public USBCANFrame20Byte {
public:
USBCANSettingsFrame(USBCANBaud baud = USB_DEF_CAN_SPEED,
                    USBCANFrameFmt frame_fmt = USB_DEF_FRAME_TYPE,
                    USBCANMode mode = USB_DEF_CAN_MODE)
	: USBCANFrame20Byte(static_cast<uint8_t>(USBCANConst::MSG_HEADER)) {

	// Initialize using safe array access
	std::vector<uint8_t> settings_data = {
		static_cast<uint8_t>(USB_DEF_FRAME_FORMAT), // Type
		static_cast<uint8_t>(baud),              // CAN baud rate
		static_cast<uint8_t>(frame_fmt),         // Frame type
		static_cast<uint8_t>(USBCANConst::DEF_FILTER0), // Filter ID
		static_cast<uint8_t>(USBCANConst::DEF_FILTER1),
		static_cast<uint8_t>(USBCANConst::DEF_FILTER2),
		static_cast<uint8_t>(USBCANConst::DEF_FILTER3),
		static_cast<uint8_t>(USBCANConst::DEF_MASK0), // Mask ID
		static_cast<uint8_t>(USBCANConst::DEF_MASK1),
		static_cast<uint8_t>(USBCANConst::DEF_MASK2),
		static_cast<uint8_t>(USBCANConst::DEF_MASK3),
		static_cast<uint8_t>(mode),                  // CAN mode
		static_cast<uint8_t>(USB_DEF_RTX),           // Auto-retransmission
		static_cast<uint8_t>(USBCANConst::RESERVED0), // Reserved
		static_cast<uint8_t>(USBCANConst::RESERVED1),
		static_cast<uint8_t>(USBCANConst::RESERVED2),
		static_cast<uint8_t>(USBCANConst::RESERVED3)
	};

	frame_data.setData(settings_data);
}

USBCANFrameType getType() const override {
	return USBCANFrameType::SETTING_FRAME;
}

// Safe parameter setters using bounds checking
void setBaudRate(USBCANBaud baud) {
	frame_data.at(1) = static_cast<uint8_t>(baud);
}

void setFrameFormat(USBCANFrameFmt fmt) {
	frame_data.at(2) = static_cast<uint8_t>(fmt);
}

void setCANMode(USBCANMode mode) {
	frame_data.at(11) = static_cast<uint8_t>(mode);
}

void setAutoRetransmit(bool enable) {
	frame_data.at(12) = enable ? 0x00 : 0x01;
}

// Safe getters
USBCANBaud getBaudRate() const {
	return static_cast<USBCANBaud>(frame_data.at(1));
}

USBCANFrameFmt getFrameFormat() const {
	return static_cast<USBCANFrameFmt>(frame_data.at(2));
}

USBCANMode getCANMode() const {
	return static_cast<USBCANMode>(frame_data.at(11));
}

bool getAutoRetransmit() const {
	return frame_data.at(12) == 0x00;
}
};

} // namespace usb_can_bridge