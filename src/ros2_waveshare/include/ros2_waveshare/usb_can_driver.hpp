/**
 * @file usb_can_driver.hpp
 * @brief USB-CAN adapter driver for ROS2
 * This file defines the USBCANDriver class which provides methods to initialize,
 * configure, send, and receive CAN frames using a USB-CAN adapter built by Waveshare.
 * The implementation is based on the USB-CAN-A example from Waveshare plus additional
 * features for ROS2 integration and error logging/handling.
 * This version will also include filtering and masking capabilities as stated in the CAN2.0 spec.
 * Conversely to the original C implementation, this C++ version encapsulates functionality
 * within a class, uses RAII for resource management, and provides a cleaner interface.
 * Also, includes utility functions for converting frame content to/from byte arrays
 * so that they are readable by human operators.
 * @author Andrea Efficace
 * @date August 2025
 */

#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace usb_can_bridge
{
// CAN frame structure
struct CANFrame
{
	uint32_t can_id; // CAN identifier
	uint8_t dlc; // Data length code (0-8)
	uint8_t data[8]; // Data payload
	bool is_extended; // Extended frame flag
	bool is_rtr; // Remote transmission request

	CANFrame() : can_id(0), dlc(0), is_extended(false), is_rtr(false)
	{
		for (int i = 0; i < 8; i++)
			data[i] = 0;
	}
};

// USB-CAN-A adapter speed enum
enum class CANSpeed
{
	SPEED_1000K = 0x01,
	SPEED_800K = 0x02,
	SPEED_500K = 0x03,
	SPEED_400K = 0x04,
	SPEED_250K = 0x05,
	SPEED_200K = 0x06,
	SPEED_125K = 0x07,
	SPEED_100K = 0x08,
	SPEED_50K = 0x09,
	SPEED_20K = 0x0A,
	SPEED_10K = 0x0B,
	SPEED_5K = 0x0C
};

// Operating modes - used by the adapter, not by the CAN bus itself
enum class CANMode
{
	NORMAL = 0x00, // This should be always used unless for testing
	LOOPBACK = 0x01,
	SILENT = 0x02,
	LOOPBACK_SILENT = 0x03
};

enum class USBBaud
{
	BAUD_9600 = 9600,
	BAUD_19200 = 19200,
	BAUD_38400 = 38400,
	BAUD_57600 = 57600,
	BAUD_115200 = 115200,
	BAUD_200000 = 2000000
};

#define USB_MIN_BAUD_RATE USBBaud::BAUD_9600
#define USB_MAX_BAUD_RATE USBBaud::BAUD_200000
#define USB_DEFAULT_BAUD_RATE USBBaud::BAUD_200000

class USBCANDriver
{
public:
USBCANDriver(const std::string& device_path, CANSpeed speed = CANSpeed::SPEED_500K,
             const USBBaud serial_baud = USB_DEFAULT_BAUD_RATE,
             rclcpp::Logger logger = rclcpp::get_logger("USBCANDriver"));
~USBCANDriver();

// Initialization
bool initialize();
bool configure(CANSpeed speed, CANMode mode = CANMode::NORMAL);
void shutdown();
void setMask(uint32_t mask);
void setFilter(uint32_t filter);
void enableDebug(bool enable)
{
	debug_enabled_ = enable;
}

// Communication
bool sendFrame(const CANFrame& frame);
bool receiveFrame(CANFrame& frame, int timeout_ms = 100);

// Status
bool isConnected() const
{
	return is_connected_;
}
std::string getLastError() const
{
	return last_error_;
}
std::string getDevicePath() const
{
	return device_path_;
}
CANSpeed getCANSpeed() const
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

// Utility functions
static CANSpeed speedFromInt(int speed_bps);
static std::string speedToString(CANSpeed speed);
void setMaskAndFilter(uint32_t mask, uint32_t filter);
void clearMaskAndFilter();

private:
std::string device_path_;
CANSpeed can_speed_;
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
bool sendSettingsCommand(CANSpeed speed, CANMode mode);
uint8_t calculateChecksum(const std::vector<uint8_t>& data);
bool isFrameComplete(const std::vector<uint8_t>& frame);

// Frame conversion
std::vector<uint8_t> frameToBytes(const CANFrame& frame);
CANSpeed speedFromInt(int speed_bps);
std::string speedToString(CANSpeed speed);
void setError(const std::string& error);
};

}  // namespace usb_can_bridge
