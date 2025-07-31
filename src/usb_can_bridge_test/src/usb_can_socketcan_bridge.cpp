#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include "usb_can_bridge_test/usb_can_driver.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <cstring>

using namespace usb_can_bridge_test;

class USBCANSocketCANBridge : public rclcpp::Node {
public:
    USBCANSocketCANBridge() : Node("usb_can_socketcan_bridge") {
        // Parameters
        this->declare_parameter("device_path", "/dev/ttyUSB0");
        this->declare_parameter("can_speed", 500000);
        this->declare_parameter("vcan_interface", "vcan0");
        this->declare_parameter("enable_debug", false);
        
        device_path_ = this->get_parameter("device_path").as_string();
        int speed_bps = this->get_parameter("can_speed").as_int();
        vcan_interface_ = this->get_parameter("vcan_interface").as_string();
        debug_enabled_ = this->get_parameter("enable_debug").as_bool();
        
        CANSpeed can_speed = USBCANDriver::speedFromInt(speed_bps);
        
        RCLCPP_INFO(this->get_logger(), "Initializing USB-CAN bridge:");
        RCLCPP_INFO(this->get_logger(), "  Device: %s", device_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Speed: %s", USBCANDriver::speedToString(can_speed).c_str());
        RCLCPP_INFO(this->get_logger(), "  Virtual CAN: %s", vcan_interface_.c_str());
        
        // Initialize USB-CAN driver
        usb_can_driver_ = std::make_unique<USBCANDriver>(device_path_, can_speed);
        
        // Setup virtual CAN interface
        if (!setupVirtualCAN()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to setup virtual CAN interface");
            return;
        }
        
        // Initialize USB-CAN adapter
        if (!usb_can_driver_->initialize()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize USB-CAN adapter: %s", 
                        usb_can_driver_->getLastError().c_str());
            return;
        }
        
        // Status publisher for debugging
        status_pub_ = this->create_publisher<std_msgs::msg::String>("bridge_status", 10);
        
        // Start bridge threads
        running_ = true;
        startBridgeThreads();
        
        RCLCPP_INFO(this->get_logger(), "USB-CAN SocketCAN bridge started successfully");
    }
    
    ~USBCANSocketCANBridge() {
        stopBridge();
    }

private:
    std::unique_ptr<USBCANDriver> usb_can_driver_;
    std::string device_path_;
    std::string vcan_interface_;
    bool debug_enabled_;
    int socketcan_fd_;
    std::atomic<bool> running_;
    
    std::thread socketcan_to_usb_thread_;
    std::thread usb_to_socketcan_thread_;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    bool setupVirtualCAN() {
        // Create virtual CAN interface if it doesn't exist
        std::string setup_cmd = "sudo modprobe vcan && "
                               "sudo ip link add dev " + vcan_interface_ + " type vcan 2>/dev/null || true && "
                               "sudo ip link set up " + vcan_interface_;
        
        int result = system(setup_cmd.c_str());
        if (result != 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to setup virtual CAN (you may need to run manually)");
        }
        
        // Open SocketCAN socket
        socketcan_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socketcan_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create SocketCAN socket");
            return false;
        }
        
        // Bind to virtual CAN interface
        struct sockaddr_can addr;
        struct ifreq ifr;
        
        strcpy(ifr.ifr_name, vcan_interface_.c_str());
        if (ioctl(socketcan_fd_, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get interface index for %s", vcan_interface_.c_str());
            close(socketcan_fd_);
            return false;
        }
        
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        
        if (bind(socketcan_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind to %s", vcan_interface_.c_str());
            close(socketcan_fd_);
            return false;
        }
        
        return true;
    }
    
    void startBridgeThreads() {
        // Thread 1: SocketCAN -> USB-CAN-A
        socketcan_to_usb_thread_ = std::thread([this]() {
            struct can_frame socketcan_frame;
            
            while (running_) {
                ssize_t nbytes = read(socketcan_fd_, &socketcan_frame, sizeof(socketcan_frame));
                
                if (nbytes > 0 && nbytes == sizeof(socketcan_frame)) {
                    // Convert SocketCAN frame to USB-CAN frame
                    CANFrame usb_frame;
                    usb_frame.can_id = socketcan_frame.can_id;
                    usb_frame.dlc = socketcan_frame.can_dlc;
                    memcpy(usb_frame.data, socketcan_frame.data, 8);
                    usb_frame.is_extended = (socketcan_frame.can_id & CAN_EFF_FLAG) != 0;
                    usb_frame.is_rtr = (socketcan_frame.can_id & CAN_RTR_FLAG) != 0;
                    
                    // Remove flags from CAN ID
                    usb_frame.can_id &= CAN_EFF_MASK;
                    
                    if (debug_enabled_) {
                        RCLCPP_INFO(this->get_logger(), "SocketCAN->USB: ID=0x%X, DLC=%d", 
                                   usb_frame.can_id, usb_frame.dlc);
                    }
                    
                    if (!usb_can_driver_->sendFrame(usb_frame)) {
                        RCLCPP_WARN(this->get_logger(), "Failed to send frame to USB-CAN: %s", 
                                   usb_can_driver_->getLastError().c_str());
                    }
                }
                
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        });
        
        // Thread 2: USB-CAN-A -> SocketCAN
        usb_to_socketcan_thread_ = std::thread([this]() {
            CANFrame usb_frame;
            
            while (running_) {
                if (usb_can_driver_->receiveFrame(usb_frame, 100)) {
                    // Convert USB-CAN frame to SocketCAN frame
                    struct can_frame socketcan_frame;
                    memset(&socketcan_frame, 0, sizeof(socketcan_frame));
                    
                    socketcan_frame.can_id = usb_frame.can_id;
                    socketcan_frame.can_dlc = usb_frame.dlc;
                    memcpy(socketcan_frame.data, usb_frame.data, 8);
                    
                    if (usb_frame.is_extended) {
                        socketcan_frame.can_id |= CAN_EFF_FLAG;
                    }
                    if (usb_frame.is_rtr) {
                        socketcan_frame.can_id |= CAN_RTR_FLAG;
                    }
                    
                    if (debug_enabled_) {
                        RCLCPP_INFO(this->get_logger(), "USB->SocketCAN: ID=0x%X, DLC=%d", 
                                   usb_frame.can_id, usb_frame.dlc);
                    }
                    
                    ssize_t bytes_written = write(socketcan_fd_, &socketcan_frame, sizeof(socketcan_frame));
                    if (bytes_written != sizeof(socketcan_frame)) {
                        RCLCPP_WARN(this->get_logger(), "Failed to write frame to SocketCAN");
                    }
                    
                    // Publish status for debugging
                    publishBridgeStatus("RX", usb_frame);
                }
            }
        });
    }
    
    void publishBridgeStatus(const std::string& direction, const CANFrame& frame) {
        auto msg = std_msgs::msg::String();
        msg.data = direction + ": ID=0x" + std::to_string(frame.can_id) + 
                  ", DLC=" + std::to_string(frame.dlc);
        status_pub_->publish(msg);
    }
    
    void stopBridge() {
        running_ = false;
        
        if (socketcan_to_usb_thread_.joinable()) {
            socketcan_to_usb_thread_.join();
        }
        
        if (usb_to_socketcan_thread_.joinable()) {
            usb_to_socketcan_thread_.join();
        }
        
        if (socketcan_fd_ >= 0) {
            close(socketcan_fd_);
        }
        
        if (usb_can_driver_) {
            usb_can_driver_->shutdown();
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<USBCANSocketCANBridge>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
