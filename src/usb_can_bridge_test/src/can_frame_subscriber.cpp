#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "usb_can_bridge_test/usb_can_driver.hpp"

using namespace usb_can_bridge_test;

class CANFrameSubscriber : public rclcpp::Node {
public:
    CANFrameSubscriber() : Node("can_frame_subscriber") {
        this->declare_parameter("device_path", "/dev/ttyUSB1");
        this->declare_parameter("can_speed", 500000);
        this->declare_parameter("enable_debug", true);
        
        device_path_ = this->get_parameter("device_path").as_string();
        int speed_bps = this->get_parameter("can_speed").as_int();
        debug_enabled_ = this->get_parameter("enable_debug").as_bool();
        
        CANSpeed can_speed = USBCANDriver::speedFromInt(speed_bps);
        
        RCLCPP_INFO(this->get_logger(), "CAN Frame Subscriber:");
        RCLCPP_INFO(this->get_logger(), "  Device: %s", device_path_.c_str());
        
        // Initialize USB-CAN driver
        usb_can_driver_ = std::make_unique<USBCANDriver>(device_path_, can_speed);
        
        if (!usb_can_driver_->initialize()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize USB-CAN adapter");
            return;
        }
        
        // Publisher for received frames
        rx_status_pub_ = this->create_publisher<std_msgs::msg::String>("can_rx_status", 10);
        
        // Start receiving thread
        running_ = true;
        receiver_thread_ = std::thread([this]() {
            CANFrame frame;
            
            while (running_) {
                if (usb_can_driver_->receiveFrame(frame, 100)) {
                    processReceivedFrame(frame);
                }
            }
        });
        
        RCLCPP_INFO(this->get_logger(), "CAN Frame Subscriber started");
    }
    
    ~CANFrameSubscriber() {
        running_ = false;
        if (receiver_thread_.joinable()) {
            receiver_thread_.join();
        }
    }

private:
    std::unique_ptr<USBCANDriver> usb_can_driver_;
    std::string device_path_;
    bool debug_enabled_;
    std::atomic<bool> running_;
    std::thread receiver_thread_;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rx_status_pub_;
    
    void processReceivedFrame(const CANFrame& frame) {
        // Parse and display frame information
        std::string frame_info = "RX: ID=0x" + std::to_string(frame.can_id) +
                               ", DLC=" + std::to_string(frame.dlc) +
                               ", Data=[";
        
        for (int i = 0; i < frame.dlc; i++) {
            if (i > 0) frame_info += " ";
            frame_info += std::to_string(frame.data[i]);
        }
        frame_info += "]";
        
        // Try to interpret as CANopen frame
        uint32_t function_code = (frame.can_id >> 7) & 0x0F;
        uint32_t node_id = frame.can_id & 0x7F;
        
        std::string interpretation = interpretCANopenFrame(frame, function_code, node_id);
        if (!interpretation.empty()) {
            frame_info += " | " + interpretation;
        }
        
        auto status_msg = std_msgs::msg::String();
        status_msg.data = frame_info;
        rx_status_pub_->publish(status_msg);
        
        if (debug_enabled_) {
            RCLCPP_INFO(this->get_logger(), "%s", frame_info.c_str());
        }
    }
    
    std::string interpretCANopenFrame(const CANFrame& frame, uint32_t function_code, uint32_t node_id) {
        std::string interpretation;
        
        switch (function_code) {
            case 0x03: // TPDO1
                interpretation = "TPDO1 from node " + std::to_string(node_id);
                if (frame.dlc >= 2) {
                    uint16_t status_word = frame.data[0] | (frame.data[1] << 8);
                    interpretation += ", Status: 0x" + std::to_string(status_word);
                }
                if (frame.dlc >= 6) {
                    int32_t velocity = frame.data[2] | (frame.data[3] << 8) | 
                                     (frame.data[4] << 16) | (frame.data[5] << 24);
                    interpretation += ", Vel: " + std::to_string(velocity * 0.001) + " rad/s";
                }
                break;
                
            case 0x04: // RPDO1
                interpretation = "RPDO1 to node " + std::to_string(node_id);
                if (frame.dlc >= 2) {
                    uint16_t control_word = frame.data[0] | (frame.data[1] << 8);
                    interpretation += ", Control: 0x" + std::to_string(control_word);
                }
                if (frame.dlc >= 6) {
                    int32_t target_vel = frame.data[2] | (frame.data[3] << 8) | 
                                       (frame.data[4] << 16) | (frame.data[5] << 24);
                    interpretation += ", Target: " + std::to_string(target_vel * 0.001) + " rad/s";
                }
                break;
                
            case 0x0B: // NMT
                interpretation = "NMT";
                break;
                
            case 0x0E: // SYNC
                interpretation = "SYNC";
                break;
                
            default:
                interpretation = "FC=" + std::to_string(function_code) + ", Node=" + std::to_string(node_id);
                break;
        }
        
        return interpretation;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<CANFrameSubscriber>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
