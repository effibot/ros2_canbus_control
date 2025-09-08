#pragma once

#include "usb_can_frames.hpp"
#include "usb_can_common.hpp"

namespace USBCANBridge
{
class FrameBuilder {
private:

public:
~FrameBuilder() = default;

FrameBuilder& setFrameType(FrameType ftype);
FrameBuilder& setFrameFormat(FrameFmt ffmt);
FrameBuilder& setID(uint32_t id);
FrameBuilder& setData(const std::vector<uint8_t>& data_bytes);
std::unique_ptr<BaseFrame> build();
};

}