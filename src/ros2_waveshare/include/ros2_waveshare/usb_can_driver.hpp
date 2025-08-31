/**
 * @file usb_can_driver.hpp
 * @brief USB-CAN adapter driver for ROS2
 * This file defines the USBCANDriver class which provides methods to initialize,
 * configure, send, and receive CAN frames using a USB-CAN adapter built by Waveshare.
 * The implementation is based on the USB-CAN-A example from Waveshare (canusb.c) and the ros2_canopen package.
 * Moreover, it includes:
 * - support for different CAN speeds and modes,
 * - support for different serial baud rates,
 * - support for both standard and extended CAN frames,
 * - support for filtering and masking (currently both zeroed in the canusb.c example),
 * - export interface to ROS2 (publish/subscribe) so that nodes can send and read DSP402 frames,
 * - additional features for ROS2 integration and error logging/handling.
 * @author Andrea Efficace
 * @date August 2025
 */

#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "usb_can_common.hpp"
namespace usb_can_bridge
{

/**
 * @brief USB-CAN-A adapter SETTING frame structure
 * This structure represents the frame format used by the USB-CAN-A adapter.
 * The complete frame consists of:
 * - Start byte (always {0xAA})
 * - Message header byte (always {0x55})
 * - Type byte (settings for fixed 20-byte protocol {0x02} or variable length protocol {0x12})
 * - CAN baud rate byte (see USBCANBaud enum)
 * - Frame type byte (standard {0x01} or extended {0x02})
 * - Filter ID[4] bytes (configure the adapter to accept only specific CAN IDs, not used in canusb.c)
 * - Mask ID[4] bytes (configure which bits of the CAN ID are relevant for filtering, not used in canusb.c)
 * - CAN Operating mode byte (normal {0x00}, loopback {0x01}, silent {0x02}, loopback+silent {0x03})
 * - Auto-retransmission byte (enabled {0x00} or disabled {0x01})
 * - Reserved bytes [4] (fixed to {0x00})
 * - Checksum byte (simple sum of bytes from Type to Reserved, modulo 256)
 * @note Two adapter can't communicate if they have different Type byte.
 * Variable length protocol is more flexible and is used in this implementation as default.
 */


struct USBCANAdapterSettingFrame {
	const uint8_t start_byte = static_cast<uint8_t>(USBCANConst::START_BYTE); // Start byte (0xAA)
	const uint8_t msg_header = static_cast<uint8_t>(USBCANConst::MSG_HEADER); // Message header byte
	const uint8_t type;     // Type byte (0x12 for variable length protocol)
	uint8_t can_baud;       // CAN baud rate (see USBCANBaud enum)
	uint8_t frame_type;     // Frame type
	std::vector<uint8_t> filter_id; // Filter ID (4 bytes)
	std::vector<uint8_t> mask_id;   // Mask ID (4 bytes)
	uint8_t can_mode;       // CAN operating mode (see USBCANMode enum)
	uint8_t auto_retransmit;   // Auto-retransmission
	const std::vector<uint8_t> reserved;      // Reserved bytes
	uint8_t checksum;       // Checksum byte

	USBCANAdapterSettingFrame(USBCANBaud baud = USB_DEF_CAN_SPEED,
	                          USBCANFrameFmt frame_fmt = USB_DEF_FRAME_TYPE,
	                          USBCANMode mode = USB_DEF_CAN_MODE,
	                          uint8_t auto_rtx = static_cast<uint8_t>(USB_DEF_RTX))
		: type(static_cast<uint8_t>(USB_DEF_FRAME_FORMAT)),
		can_baud(static_cast<uint8_t>(baud)),
		frame_type(static_cast<uint8_t>(frame_fmt)),
		filter_id({static_cast<uint8_t>(USBCANConst::DEF_FILTER0),
		           static_cast<uint8_t>(USBCANConst::DEF_FILTER1),
		           static_cast<uint8_t>(USBCANConst::DEF_FILTER2),
		           static_cast<uint8_t>(USBCANConst::DEF_FILTER3)}),
		mask_id({static_cast<uint8_t>(USBCANConst::DEF_MASK0),
		         static_cast<uint8_t>(USBCANConst::DEF_MASK1),
		         static_cast<uint8_t>(USBCANConst::DEF_MASK2),
		         static_cast<uint8_t>(USBCANConst::DEF_MASK3)}),
		can_mode(static_cast<uint8_t>(mode)),
		auto_retransmit(auto_rtx),
		reserved({static_cast<uint8_t>(USBCANConst::RESERVED0),
		          static_cast<uint8_t>(USBCANConst::RESERVED1),
		          static_cast<uint8_t>(USBCANConst::RESERVED2),
		          static_cast<uint8_t>(USBCANConst::RESERVED3)}),
		checksum(0) {
		//TODO Calculate checksum
		checksum = 0x00;
	}
};

/**
 * @brief CAN Adapter frame structure
 * This structure represents the CAN frame as used by the USB-CAN-A adapter.
 * The adapter can use both standard (11-bit ID) and extended (29-bit ID) frames
 * but the protocol supports both fixed (20 bytes) and variable length frames.
 * The implementation uses an abstract CANFrame structure to represent both types and
 * a factory method to convert between the abstract and adapter-specific formats.
 * For the fixed frame format, the complete frame consists of:
 * - Start byte (always {0xAA})
 * - Message header byte (always {0x55})
 * - Type byte {0x01}
 * - Framework type byte {0x01}
 * - Frame ID [4] bytes, little-endian
 * - Frame Data Length Code (DLC) byte (0-8)
 * - Frame Data [8] bytes (padded with zeros if DLC < 8)
 * - Reserved byte (always {0x00})
 * - Checksum Code (The low 8 bits of the cumulative sum from frame type to error code )
 *
 * For the variable length frame format, the complete frame consists of:
 * - Start byte (always {0xAA})
 * - Type byte:
 * 	- [Frame Type] bit 5: 0 for standard frame (2 bytes ID), 1 for extended frame (4 bytes ID)
 * 	- [Frame Format] bit 4: 0 for data frame, 1 for remote frame
 * 	- [DLC] bits 3-0: Data Length Code (0-8)
 * - Frame ID [2 or 4] bytes, little-endian
 * - Frame Data [0-8] bytes
 * - End byte (always {0x55})
 * @note The variable length frame format is more flexible and is used in this implementation as default
 */
struct USBCANAdapterBaseFrame {

};


#define MIN_CONF_SLEEP_MS 1000                  // Minimum sleep time after configuration commands (in milliseconds)

class USBCANDriver
{
public:
USBCANDriver(const std::string& device_path, USBCANBaud speed = USB_DEF_CAN_SPEED,
             const USBBaud serial_baud = USB_DEF_BAUD_RATE,
             rclcpp::Logger logger = rclcpp::get_logger("USBCANDriver"));
~USBCANDriver();

// Initialization
bool initialize();
bool configureCANBus(USBCANBaud speed = USB_DEF_CAN_SPEED, USBCANMode mode = USB_DEF_CAN_MODE);
void shutdown();
void setMask(uint32_t mask);
void setFilter(uint32_t filter);
void enableDebug(bool enable)
{
	debug_enabled_ = enable;
}

// Communication
//bool sendFrame(const CANFrame& frame);
//bool receiveFrame(CANFrame& frame, int timeout_ms = 100);

// Status
bool isConnected() const
{
	return is_connected_;
}
std::string getLastError() const
{
	return last_error_;
}

// Utility functions
static USBCANBaud CANSpeedFromInt(int speed_bps);
std::string getDevicePath() const
{
	return device_path_;
}
USBCANBaud getCANSpeed() const
{
	return can_speed_;
}
rclcpp::Logger getLogger() const
{
	return logger_;
}
USBBaud getSerialBaudRate() const
{
	return serial_baud_;
}
void setSerialBaudRate(USBBaud baud_rate)
{
	serial_baud_ = baud_rate;

}
USBCANBaud speedFromInt(int speed_bps);
std::string speedToString(USBCANBaud speed);

private:
std::string device_path_;
USBCANBaud can_speed_;
int serial_fd_;
bool is_connected_;
std::string last_error_;
rclcpp::Logger logger_;
bool debug_enabled_ = false;
USBBaud serial_baud_;

// Low-level serial communication
bool openSerial();
bool closeSerial();
bool configureSerial();

// Protocol handling
bool sendCommand(const std::vector<uint8_t>& command);
bool receiveResponse(std::vector<uint8_t>& response, int timeout_ms);
bool sendSettingsCommand(USBCANBaud speed, USBCANMode mode);
uint8_t calculateChecksum(const std::vector<uint8_t>& data);
bool isFrameComplete(const std::vector<uint8_t>& frame);
void setMaskAndFilter(uint32_t mask, uint32_t filter);
void clearMaskAndFilter();

// Frame conversion
//std::vector<uint8_t> frameToBytes(const CANFrame& frame);
//bool bytesToFrame(const std::vector<uint8_t>& bytes, CANFrame& frame);

// Error handling
void setError(const std::string& error){
	last_error_ = error;
	if (debug_enabled_)
	{
		RCLCPP_ERROR(logger_, "%s", error.c_str());
	}
}
};

}  // namespace usb_can_bridge
