/**
 * @file frame_factory.cpp
 * @brief Implementation of Factory class for creating USB-CAN frame objects
 *
 * @author Andrea Efficace
 * @date September 2025
 */

#include "ros2_waveshare/frame_factory.hpp"
#include <stdexcept>

namespace USBCANBridge
{

std::unique_ptr<FixedSizeFrame> FrameFactory::createFixedFrame(
	Type type, FrameType frame_type, FrameFmt fmt) {
	return std::make_unique<FixedSizeFrame>(type, frame_type, fmt);
}

std::unique_ptr<VariableSizeFrame> FrameFactory::createVariableFrame(
	FrameType frame_type, FrameFmt frame_fmt, uint8_t dlc) {
	return std::make_unique<VariableSizeFrame>(frame_type, frame_fmt, dlc);
}

std::unique_ptr<AdapterBaseFrame> FrameFactory::createFrameFromData(
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

} // namespace USBCANBridge
