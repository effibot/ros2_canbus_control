#pragma once

#include "usb_can_frames.hpp"
#include "usb_can_common.hpp"

namespace USBCANBridge
{
class USBCANFrameBuilder {

public:
~USBCANFrameBuilder() = default;

USBCANFrameBuilder& setFrameType(FrameType ftype);
USBCANFrameBuilder& setFrameFormat(FrameFmt frame_fmt);
USBCANFrameBuilder& setID(uint32_t id);
USBCANFrameBuilder& setData(const std::vector<uint8_t>& data_bytes);
std::unique_ptr<BaseFrame> build();
};

}