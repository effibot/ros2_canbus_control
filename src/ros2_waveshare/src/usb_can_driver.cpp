#include "include/ros2_waveshare/usb_can_driver.hpp"
#include <rclcpp/rclcpp.hpp>

namespace ros2_waveshare
{
USBCANDriver::USBCANDriver(const std::string& device_path, CANSpeed speed)
  : device_path_(device_path), can_speed_(speed), serial_fd_(-1), is_connected_(false)
{
}

USBCANDriver::~USBCANDriver()
{
  shutdown();
}
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
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  if (!configure(can_speed_, CANMode::NORMAL))
  {
    RCLCPP_ERROR(logger_, "Failed to configure CAN settings");
    closeSerial();
    return false;
  }

  is_connected_ = true;
  RCLCPP_INFO(logger_, "USB-CAN device initialized on %s at %s", device_path_.c_str(),
              speedToString(can_speed_).c_str());
  return true;
}
