#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "usb_can_bridge_test/usb_can_driver.hpp"
#include <chrono>

using namespace usb_can_bridge_test;

class CANFramePublisher : public rclcpp::Node {
public:
    CANFramePublisher() : Node("can_frame_publisher") {
        this->declare_parameter("device_path", "/dev/ttyUSB0");
        this->declare_parameter("can_speed", 500000);
        this->declare_parameter("target_node_id", 2);
        this->declare_parameter("publish_rate", 10.0);
        
        device_path_ = this->get_parameter("device_path").as_string();
        int speed_bps = this->get_parameter("can_speed").as_int();
        target_node_id_ = this->get_parameter("target_node_id").as_int();
        double rate = this->get_parameter("publish_rate").as_double();
        
        CANSpeed can_speed = USBCANDriver::speedFromInt(speed_bps);
        
        RCLCPP_INFO(this->get_logger(), "CAN Frame Publisher:");
        RCLCPP_INFO(this->get_logger(), "  Device: %s", device_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Target Node ID: %d", target_node_id_);
        
        // Initialize USB-CAN driver
        usb_can_driver_ = std::make_unique<USBCANDriver>(device_path_, can_speed);
        
        if (!usb_can_driver_->initialize()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize USB-CAN adapter");
            return;
        }
        
        // Publisher for status
        status_pub_ = this->create_publisher<std_msgs::msg::String>("can_tx_status", 10);
        
        // Timer for sending frames
        timer_ = this->create_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
            std::bind(&CANFramePublisher::sendFrame, this));
        
        target_velocity_ = 0.0;
        control_word_ = 0x000F; // Enable motor
    }

private:
    std::unique_ptr<USBCANDriver> usb_can_driver_;
    std::string device_path_;
    int target_node_id_;
    double target_velocity_;
    uint16_t control_word_;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    void sendFrame() {
        // Create CANopen RPDO1 frame (control word + target velocity)
        CANFrame frame;
        frame.can_id = 0x200 + target_node_id_; // RPDO1 COB-ID
        frame.dlc = 6;
        frame.is_extended = false;
        frame.is_rtr = false;
        
        // Generate varying target velocity (sine wave)
        static double time = 0.0;
        time += 0.1;
        target_velocity_ = 3.0 * sin(time); // ±3 rad/s
        
        // Control word (16-bit)
        frame.data[0] = control_word_ & 0xFF;
        frame.data[1] = (control_word_ >> 8) & 0xFF;
        
        // Target velocity (32-bit, scaled)
        int32_t velocity_scaled = static_cast<int32_t>(target_velocity_ * 1000);
        frame.data[2] = velocity_scaled & 0xFF;
        frame.data[3] = (velocity_scaled >> 8) & 0xFF;
        frame.data[4] = (velocity_scaled >> 16) & 0xFF;
        frame.data[5] = (velocity_scaled >> 24) & 0xFF;
        
        if (usb_can_driver_->sendFrame(frame)) {
            auto status_msg = std_msgs::msg::String();
            status_msg.data = "Sent: ID=0x" + std::to_string(frame.can_id) + 
                             ", Target Vel=" + std::to_string(target_velocity_);
            status_pub_->publish(status_msg);
            
            RCLCPP_INFO(this->get_logger(), "Sent velocity command: %.3f rad/s", target_velocity_);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to send frame: %s", 
                       usb_can_driver_->getLastError().c_str());
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<CANFramePublisher>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
