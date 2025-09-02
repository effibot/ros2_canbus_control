#pragma once

#include "usb_can_frame.hpp"
#include "usb_can_common.hpp"

namespace usb_can_bridge
{
class USBCANFrameBuilder {

public:
~USBCANFrameBuilder() = default;

USBCANFrameBuilder& setFrameType(USBCANFrameType frame_type);
USBCANFrameBuilder& setFrameFormat(USBCANFrameFmt frame_fmt);
USBCANFrameBuilder& setID(uint32_t id);
USBCANFrameBuilder& setData(const std::vector<uint8_t>& data_bytes);
std::unique_ptr<USBCANAdapterBaseFrame> build();
};

}