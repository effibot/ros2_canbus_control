#pragma once

#include "usb_can_common.hpp"
#include "usb_can_frame.hpp"
#include <map>

namespace usb_can_bridge {

class USBCANFrameFactory {

public:
virtual ~USBCANFrameFactory() = default;
virtual std::unique_ptr<USBCANAdapterBaseFrame> createFrame(const std::vector<uint8_t>& raw_data) = 0;
virtual std::unique_ptr<USBCANAdapterBaseFrame> createFrameFromData(const std::vector<uint8_t>& data) = 0;
};

class USBCANFrameFactoryManager {
private:
std::map<USBCANFrameType, std::unique_ptr<USBCANFrameFactory> > factories_;
public:
std::unique_ptr<USBCANAdapterBaseFrame> createFrame(USBCANFrameType frame_type);
std::unique_ptr<USBCANAdapterBaseFrame> createFrameFromData(const std::vector<uint8_t>& data);
};

}
