#include "usb_can_bridge_test/usb_can_driver.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <cstring>
#include <errno.h>
#include <iostream>
#include <chrono>
#include <thread>

namespace usb_can_bridge_test {

USBCANDriver::USBCANDriver(const std::string& device_path, CANSpeed speed)
    : device_path_(device_path), can_speed_(speed), serial_fd_(-1), is_connected_(false) {
}

USBCANDriver::~USBCANDriver() {
    shutdown();
}

bool USBCANDriver::initialize() {
    if (!openSerial()) {
        return false;
    }
    
    if (!configureSerial()) {
        closeSerial();
        return false;
    }
    
    // Wait for device to settle
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    if (!configure(can_speed_, CANMode::NORMAL)) {
        closeSerial();
        return false;
    }
    
    is_connected_ = true;
    return true;
}

bool USBCANDriver::configure(CANSpeed speed, CANMode mode) {
    can_speed_ = speed;
    return sendSettingsCommand(speed, mode);
}

void USBCANDriver::shutdown() {
    if (is_connected_) {
        closeSerial();
        is_connected_ = false;
    }
}

bool USBCANDriver::sendFrame(const CANFrame& frame) {
    if (!is_connected_) {
        setError("Device not connected");
        return false;
    }
    
    auto bytes = frameToBytes(frame);
    return sendCommand(bytes);
}

bool USBCANDriver::receiveFrame(CANFrame& frame, int timeout_ms) {
    if (!is_connected_) {
        setError("Device not connected");
        return false;
    }
    
    std::vector<uint8_t> response;
    if (!receiveResponse(response, timeout_ms)) {
        return false;
    }
    
    return bytesToFrame(response, frame);
}

bool USBCANDriver::openSerial() {
    serial_fd_ = open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ == -1) {
        std::string error_msg = "Failed to open device: " + device_path_ + 
                               " (errno: " + std::to_string(errno) + " - " + strerror(errno) + ")";
        setError(error_msg);
        return false;
    }
    return true;
}

void USBCANDriver::closeSerial() {
    if (serial_fd_ != -1) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

bool USBCANDriver::configureSerial() {
    struct termios2 tio;
    
    if (ioctl(serial_fd_, TCGETS2, &tio) == -1) {
        setError("Failed to get serial configuration");
        return false;
    }
    
    // Configure for USB-CAN-A protocol
    tio.c_cflag &= ~CBAUD;
    tio.c_cflag = BOTHER | CS8 | CSTOPB;  // 8 data bits, 2 stop bits
    tio.c_iflag = IGNPAR;                 // Ignore parity
    tio.c_oflag = 0;                      // No output processing
    tio.c_lflag = 0;                      // No line processing
    tio.c_ispeed = 2000000;               // 2 Mbps input speed
    tio.c_ospeed = 2000000;               // 2 Mbps output speed
    
    if (ioctl(serial_fd_, TCSETS2, &tio) == -1) {
        setError("Failed to set serial configuration");
        return false;
    }
    
    return true;
}

bool USBCANDriver::sendCommand(const std::vector<uint8_t>& command) {
    if (serial_fd_ == -1) {
        setError("Serial port not open");
        return false;
    }
    
    ssize_t bytes_written = write(serial_fd_, command.data(), command.size());
    if (bytes_written != static_cast<ssize_t>(command.size())) {
        setError("Failed to write complete command");
        return false;
    }
    
    return true;
}

bool USBCANDriver::receiveResponse(std::vector<uint8_t>& response, int timeout_ms) {
    response.clear();
    
    auto start_time = std::chrono::steady_clock::now();
    auto timeout = std::chrono::milliseconds(timeout_ms);
    
    while (std::chrono::steady_clock::now() - start_time < timeout) {
        uint8_t byte;
        ssize_t bytes_read = read(serial_fd_, &byte, 1);
        
        if (bytes_read > 0) {
            response.push_back(byte);
            
            if (isFrameComplete(response)) {
                return true;
            }
        } else if (bytes_read == -1 && errno != EAGAIN && errno != EWOULDBLOCK) {
            setError("Serial read error");
            return false;
        }
        
        // Small delay to prevent busy waiting
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    
    return false; // Timeout
}

bool USBCANDriver::sendSettingsCommand(CANSpeed speed, CANMode mode) {
    std::vector<uint8_t> cmd(20);
    
    // Build settings command frame
    cmd[0] = 0xAA;                    // Frame start
    cmd[1] = 0x55;                    // Command frame
    cmd[2] = 0x12;                    // Settings command
    cmd[3] = static_cast<uint8_t>(speed);  // CAN speed
    cmd[4] = 0x01;                    // Standard frame type
    cmd[5] = cmd[6] = cmd[7] = cmd[8] = 0;  // Filter ID (not used)
    cmd[9] = cmd[10] = cmd[11] = cmd[12] = 0;  // Mask ID (not used)
    cmd[13] = static_cast<uint8_t>(mode);   // Operating mode
    cmd[14] = 0x01;                   // Fixed value
    cmd[15] = cmd[16] = cmd[17] = cmd[18] = 0;  // Reserved
    cmd[19] = calculateChecksum(std::vector<uint8_t>(cmd.begin() + 2, cmd.begin() + 19));
    
    return sendCommand(cmd);
}

std::vector<uint8_t> USBCANDriver::frameToBytes(const CANFrame& frame) {
    std::vector<uint8_t> bytes;
    
    // Data frame format
    bytes.push_back(0xAA);  // Frame start
    
    // Frame info byte
    uint8_t info = 0xC0;  // Data frame bits (7:6 = 11)
    if (frame.is_extended) {
        info |= 0x20;  // Extended frame
    }
    if (frame.is_rtr) {
        info |= 0x10;  // RTR frame
    }
    info |= (frame.dlc & 0x0F);  // DLC
    bytes.push_back(info);
    
    // CAN ID (little endian)
    bytes.push_back(frame.can_id & 0xFF);         // LSB
    bytes.push_back((frame.can_id >> 8) & 0xFF);  // MSB (for standard frames)
    
    // Data payload
    for (int i = 0; i < frame.dlc && i < 8; i++) {
        bytes.push_back(frame.data[i]);
    }
    
    // Frame end
    bytes.push_back(0x55);
    
    return bytes;
}

bool USBCANDriver::bytesToFrame(const std::vector<uint8_t>& bytes, CANFrame& frame) {
    if (bytes.size() < 6) {
        setError("Frame too short");
        return false;
    }
    
    if (bytes[0] != 0xAA) {
        setError("Invalid frame start marker");
        return false;
    }
    
    // Parse frame info
    uint8_t info = bytes[1];
    if ((info & 0xC0) != 0xC0) {
        setError("Not a data frame");
        return false;
    }
    
    frame.is_extended = (info & 0x20) != 0;
    frame.is_rtr = (info & 0x10) != 0;
    frame.dlc = info & 0x0F;
    
    // Parse CAN ID
    frame.can_id = bytes[2] | (bytes[3] << 8);
    
    // Parse data
    for (int i = 0; i < frame.dlc && i < 8; i++) {
        if (4 + i < bytes.size()) {
            frame.data[i] = bytes[4 + i];
        }
    }
    
    return true;
}

uint8_t USBCANDriver::calculateChecksum(const std::vector<uint8_t>& data) {
    uint8_t checksum = 0;
    for (uint8_t byte : data) {
        checksum += byte;
    }
    return checksum;
}

bool USBCANDriver::isFrameComplete(const std::vector<uint8_t>& frame) {
    if (frame.empty()) return false;
    
    if (frame[0] != 0xAA) {
        return true;  // Skip to next frame
    }
    
    if (frame.size() < 2) return false;
    
    if (frame[1] == 0x55) {
        // Command frame - always 20 bytes
        return frame.size() >= 20;
    } else if ((frame[1] & 0xF0) == 0xC0) {
        // Data frame - variable length
        uint8_t dlc = frame[1] & 0x0F;
        return frame.size() >= (5 + dlc);
    }
    
    return true;  // Unknown frame type
}

CANSpeed USBCANDriver::speedFromInt(int speed_bps) {
    switch (speed_bps) {
        case 1000000: return CANSpeed::SPEED_1000K;
        case 800000:  return CANSpeed::SPEED_800K;
        case 500000:  return CANSpeed::SPEED_500K;
        case 400000:  return CANSpeed::SPEED_400K;
        case 250000:  return CANSpeed::SPEED_250K;
        case 200000:  return CANSpeed::SPEED_200K;
        case 125000:  return CANSpeed::SPEED_125K;
        case 100000:  return CANSpeed::SPEED_100K;
        case 50000:   return CANSpeed::SPEED_50K;
        case 20000:   return CANSpeed::SPEED_20K;
        case 10000:   return CANSpeed::SPEED_10K;
        case 5000:    return CANSpeed::SPEED_5K;
        default:      return CANSpeed::SPEED_500K;
    }
}

std::string USBCANDriver::speedToString(CANSpeed speed) {
    switch (speed) {
        case CANSpeed::SPEED_1000K: return "1000K";
        case CANSpeed::SPEED_800K:  return "800K";
        case CANSpeed::SPEED_500K:  return "500K";
        case CANSpeed::SPEED_400K:  return "400K";
        case CANSpeed::SPEED_250K:  return "250K";
        case CANSpeed::SPEED_200K:  return "200K";
        case CANSpeed::SPEED_125K:  return "125K";
        case CANSpeed::SPEED_100K:  return "100K";
        case CANSpeed::SPEED_50K:   return "50K";
        case CANSpeed::SPEED_20K:   return "20K";
        case CANSpeed::SPEED_10K:   return "10K";
        case CANSpeed::SPEED_5K:    return "5K";
        default:                    return "Unknown";
    }
}

void USBCANDriver::setError(const std::string& error) {
    last_error_ = error;
    std::cerr << "USB-CAN Error: " << error << std::endl;
}

} // namespace usb_can_bridge_test
