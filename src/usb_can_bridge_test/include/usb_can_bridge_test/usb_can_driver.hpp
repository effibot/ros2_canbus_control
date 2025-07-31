#ifndef USB_CAN_DRIVER_HPP
#define USB_CAN_DRIVER_HPP

#include <string>
#include <vector>
#include <cstdint>
#include <memory>

namespace usb_can_bridge_test {

// CAN frame structure
struct CANFrame {
    uint32_t can_id;        // CAN identifier
    uint8_t dlc;           // Data length code (0-8)
    uint8_t data[8];       // Data payload
    bool is_extended;      // Extended frame flag
    bool is_rtr;           // Remote transmission request
    
    CANFrame() : can_id(0), dlc(0), is_extended(false), is_rtr(false) {
        for (int i = 0; i < 8; i++) data[i] = 0;
    }
};

// USB-CAN-A adapter speed enum
enum class CANSpeed {
    SPEED_1000K = 0x01,
    SPEED_800K  = 0x02,
    SPEED_500K  = 0x03,
    SPEED_400K  = 0x04,
    SPEED_250K  = 0x05,
    SPEED_200K  = 0x06,
    SPEED_125K  = 0x07,
    SPEED_100K  = 0x08,
    SPEED_50K   = 0x09,
    SPEED_20K   = 0x0A,
    SPEED_10K   = 0x0B,
    SPEED_5K    = 0x0C
};

// Operating modes
enum class CANMode {
    NORMAL = 0x00,
    LOOPBACK = 0x01,
    SILENT = 0x02,
    LOOPBACK_SILENT = 0x03
};

class USBCANDriver {
public:
    USBCANDriver(const std::string& device_path, CANSpeed speed = CANSpeed::SPEED_500K);
    ~USBCANDriver();
    
    // Initialization
    bool initialize();
    bool configure(CANSpeed speed, CANMode mode = CANMode::NORMAL);
    void shutdown();
    
    // Communication
    bool sendFrame(const CANFrame& frame);
    bool receiveFrame(CANFrame& frame, int timeout_ms = 100);
    
    // Status
    bool isConnected() const { return is_connected_; }
    std::string getLastError() const { return last_error_; }
    
    // Utility functions
    static CANSpeed speedFromInt(int speed_bps);
    static std::string speedToString(CANSpeed speed);

private:
    std::string device_path_;
    CANSpeed can_speed_;
    int serial_fd_;
    bool is_connected_;
    std::string last_error_;
    
    // Low-level serial communication
    bool openSerial();
    void closeSerial();
    bool configureSerial();
    
    // Protocol handling
    bool sendCommand(const std::vector<uint8_t>& command);
    bool receiveResponse(std::vector<uint8_t>& response, int timeout_ms);
    bool sendSettingsCommand(CANSpeed speed, CANMode mode);
    
    // Frame conversion
    std::vector<uint8_t> frameToBytes(const CANFrame& frame);
    bool bytesToFrame(const std::vector<uint8_t>& bytes, CANFrame& frame);
    
    // Utility functions
    uint8_t calculateChecksum(const std::vector<uint8_t>& data);
    bool isFrameComplete(const std::vector<uint8_t>& frame);
    void setError(const std::string& error);
};

} // namespace usb_can_bridge_test

#endif // USB_CAN_DRIVER_HPP
