/**
 * @file frame_factory.hpp
 * @brief Factory class for creating USB-CAN frame objects
 * This file defines the FrameFactory class that provides static methods to create
 * frame objects using the builder pattern. It ensures proper initialization and
 * validation of frame parameters.
 *
 * @author Andrea Efficace
 * @date September 2025
 */

#pragma once

#include "adapter_base_frame.hpp"
#include "fixed_size_frame.hpp"
#include "variable_size_frame.hpp"
#include "usb_can_common.hpp"

#include <memory>
#include <vector>
#include <stdexcept>

namespace USBCANBridge
{

/**
 * @brief Factory class for creating frame objects
 * This class provides static methods to create frame objects using the builder pattern.
 * It ensures proper initialization and validation of frame parameters.
 */
class FrameFactory {
public:
// Create fixed size frame
static std::unique_ptr<FixedSizeFrame> createFixedFrame(
	Type type, FrameType frame_type, FrameFmt fmt) {
	return std::make_unique<FixedSizeFrame>(type, frame_type, fmt);
}

// Create variable size frame
static std::unique_ptr<VariableSizeFrame> createVariableFrame(
	FrameType frame_type, FrameFmt frame_fmt, uint8_t dlc = 0) {
	return std::make_unique<VariableSizeFrame>(frame_type, frame_fmt, dlc);
}

// Create frame from raw data (auto-detect type)
static std::unique_ptr<AdapterBaseFrame> createFrameFromData(
	const std::vector<uint8_t>& raw_data) {

	if (raw_data.empty()) {
		throw std::invalid_argument("Empty data provided");
	}

	if (raw_data[0] != to_uint8(Constants::START_BYTE)) {
		throw std::invalid_argument("Invalid start byte");
	}

	// Determine frame type based on size and structure
	if (raw_data.size() == 20 && raw_data[1] == to_uint8(Constants::MSG_HEADER)) {
		// Fixed frame
		auto frame = std::make_unique<FixedSizeFrame>();
		if (frame->deserialize(raw_data)) {
			return frame;
		}
	} else if (raw_data.size() >= 6 && raw_data.size() <= 15 &&
	           raw_data[raw_data.size() - 1] == to_uint8(Constants::END_BYTE)) {
		// Variable frame
		auto frame = std::make_unique<VariableSizeFrame>();
		if (frame->deserialize(raw_data)) {
			return frame;
		}
	}

	throw std::invalid_argument("Unable to parse frame data");
}

};

} // namespace USBCANBridge
