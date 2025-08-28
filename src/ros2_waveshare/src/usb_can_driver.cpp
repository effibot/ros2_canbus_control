#include "ros2_waveshare/usb_can_driver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <cstring>
#include <errno.h>
#include <iostream>
#include <chrono>
#include <thread>
#include "usb_can_driver.hpp"
namespace usb_can_bridge {

// Constructor
USBCANDriver::USBCANDriver(const std::string& device_path, CANSpeed speed, const USBBaud serial_baud, rclcpp::Logger logger)
	: device_path_(device_path), can_speed_(speed), serial_fd_(-1), is_connected_(false),
	logger_(logger), serial_baud_(serial_baud) {
}
// Destructor
USBCANDriver::~USBCANDriver() {
	shutdown();
}
//! -- Drover Initialization --
bool USBCANDriver::initialize()
{
	if (!openSerial())
	{
		RCLCPP_ERROR(logger_, "Failed to open serial port");
		return false;
	}

	if (!configureSerial())
	{
		RCLCPP_ERROR(logger_, "Failed to configure serial port");
		closeSerial();
		return false;
	}

	// Wait for device to settle
	std::this_thread::sleep_for(std::chrono::milliseconds(MIN_CONF_SLEEP_MS));

	if (!configureCANBus(can_speed_, USBMode::NORMAL))
	{
		RCLCPP_ERROR(logger_, "Failed to configure CAN settings");
		closeSerial();
		return false;
	}
	is_connected_ = true;
	RCLCPP_INFO(logger_, "USB-CAN device on %s connected at %s baud", device_path_.c_str(),
	            speedToString(can_speed_).c_str());
	return true;
}

//! -- Serial Configuration --
// Open serial port
bool USBCANDriver::openSerial()
{
	serial_fd_ = open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (serial_fd_ == -1)
	{
		std::string error_msg = "Failed to open device: " + device_path_ +
		                        " (errno: " + std::to_string(errno) + " - " + strerror(errno) + ")";
		setError(error_msg);
		return false;
	}
	return true;
}
// Close serial port
bool USBCANDriver::closeSerial()
{
	if (serial_fd_ != -1)
	{
		close(serial_fd_);
		serial_fd_ = -1;
	}
	return true;
}
// Shutdown and cleanup
void USBCANDriver::shutdown() {
	if (is_connected_) {
		closeSerial();
		is_connected_ = false;
		RCLCPP_INFO(logger_, "USB-CAN device on %s disconnected", device_path_.c_str());
	}
}
// Configure USB transducer settings
bool USBCANDriver::configureSerial()
{
	struct termios2 tio;

	if (ioctl(serial_fd_, TCGETS2, &tio) == -1)
	{
		setError("Failed to get serial configuration");
		return false;
	}

	// Configure for USB-CAN-A protocol
	tio.c_cflag &= ~CBAUD;
	tio.c_cflag = BOTHER | CS8 | CSTOPB; // 8 data bits, 2 stop bits
	tio.c_iflag = IGNPAR;                // Ignore parity
	tio.c_oflag = 0;                     // No output processing
	tio.c_lflag = 0;                     // No line processing
	tio.c_ispeed = static_cast<int>(serial_baud_); // Input speed
	tio.c_ospeed = static_cast<int>(serial_baud_); // Output speed

	if (ioctl(serial_fd_, TCSETS2, &tio) == -1)
	{
		setError("Failed to set serial configuration");
		return false;
	}

	return true;
}

//! -- CAN Bus Configuration --
// Configure CAN bus settings
bool USBCANDriver::configureCANBus(CANSpeed speed, USBMode mode)
{
	can_speed_ = speed;
	return sendSettingsCommand(speed, mode);
}
// set CAN bus mask
void USBCANDriver::setMask(uint32_t mask)
{
	return;
}
// set CAN bus filter
void USBCANDriver::setFilter(uint32_t filter)
{
	return;
}

} // namespace usb_can_bridge
