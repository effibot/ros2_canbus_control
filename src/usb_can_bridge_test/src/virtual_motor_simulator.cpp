#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include "usb_can_bridge_test/usb_can_driver.hpp"
#include <thread>
#include <atomic>
#include <cmath>

using namespace usb_can_bridge_test;

class VirtualMotorSimulator : public rclcpp::Node {
public:
    VirtualMotorSimulator() : Node("virtual_motor_simulator") {
        // Parameters
        this->declare_parameter("device_path", "/dev/ttyUSB1");
        this->declare_parameter("can_speed", 500000);
        this->declare_parameter("node_id", 2);
        this->declare_parameter("enable_debug", false);
        this->declare_parameter("max_velocity", 10.0);  // rad/s
        this->declare_parameter("update_rate", 50.0);   // Hz
        
        device_path_ = this->get_parameter("device_path").as_string();
        int speed_bps = this->get_parameter("can_speed").as_int();
        canopen_node_id_ = this->get_parameter("node_id").as_int();
        debug_enabled_ = this->get_parameter("enable_debug").as_bool();
        max_velocity_ = this->get_parameter("max_velocity").as_double();
        update_rate_ = this->get_parameter("update_rate").as_double();
        
        CANSpeed can_speed = USBCANDriver::speedFromInt(speed_bps);
        
        RCLCPP_INFO(this->get_logger(), "Initializing Virtual Motor Simulator:");
        RCLCPP_INFO(this->get_logger(), "  Device: %s", device_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Speed: %s", USBCANDriver::speedToString(can_speed).c_str());
        RCLCPP_INFO(this->get_logger(), "  CANopen Node ID: %d", canopen_node_id_);
        
        // Initialize USB-CAN driver
        usb_can_driver_ = std::make_unique<USBCANDriver>(device_path_, can_speed);
        
        if (!usb_can_driver_->initialize()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize USB-CAN adapter: %s", 
                        usb_can_driver_->getLastError().c_str());
            return;
        }
        
        // Initialize motor state
        motor_state_.position = 0.0;
        motor_state_.velocity = 0.0;
        motor_state_.target_velocity = 0.0;
        motor_state_.enabled = false;
        motor_state_.status_word = 0x0000;
        motor_state_.control_word = 0x0000;
        
        // Publishers
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("motor_status", 10);
        turtle_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        
        // Start simulation threads
        running_ = true;
        startSimulation();
        
        RCLCPP_INFO(this->get_logger(), "Virtual Motor Simulator started successfully");
    }
    
    ~VirtualMotorSimulator() {
        stopSimulation();
    }

private:
    struct MotorState {
        double position;         // rad
        double velocity;         // rad/s
        double target_velocity;  // rad/s
        bool enabled;
        uint16_t status_word;    // CANopen status word
        uint16_t control_word;   // CANopen control word
    };
    
    std::unique_ptr<USBCANDriver> usb_can_driver_;
    std::string device_path_;
    int canopen_node_id_;
    bool debug_enabled_;
    double max_velocity_;
    double update_rate_;
    std::atomic<bool> running_;
    
    MotorState motor_state_;
    std::mutex motor_state_mutex_;
    
    std::thread can_receiver_thread_;
    std::thread motor_simulation_thread_;
    
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle_cmd_pub_;
    
    void startSimulation() {
        // Thread 1: Receive CAN commands
        can_receiver_thread_ = std::thread([this]() {
            CANFrame frame;
            
            while (running_) {
                if (usb_can_driver_->receiveFrame(frame, 100)) {
                    processCANFrame(frame);
                }
            }
        });
        
        // Thread 2: Motor simulation and status transmission
        motor_simulation_thread_ = std::thread([this]() {
            auto last_update = std::chrono::steady_clock::now();
            double dt = 1.0 / update_rate_;
            
            while (running_) {
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration<double>(now - last_update).count();
                
                if (elapsed >= dt) {
                    updateMotorSimulation(elapsed);
                    sendCANopenStatus();
                    publishROS2Status();
                    publishTurtleCommand();
                    last_update = now;
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });
    }
    
    void processCANFrame(const CANFrame& frame) {
        // Parse CANopen messages
        uint32_t function_code = (frame.can_id >> 7) & 0x0F;
        uint32_t node_id = frame.can_id & 0x7F;
        
        if (node_id != static_cast<uint32_t>(canopen_node_id_)) {
            return; // Not for this node
        }
        
        std::lock_guard<std::mutex> lock(motor_state_mutex_);
        
        switch (function_code) {
            case 0x04: // RPDO1 (typically control word + target velocity)
                if (frame.dlc >= 2) {
                    motor_state_.control_word = frame.data[0] | (frame.data[1] << 8);
                    
                    // Check enable bit
                    motor_state_.enabled = (motor_state_.control_word & 0x0F) == 0x0F;
                    
                    if (frame.dlc >= 6) {
                        // Target velocity (32-bit signed integer)
                        int32_t target_vel_raw = frame.data[2] | (frame.data[3] << 8) | 
                                               (frame.data[4] << 16) | (frame.data[5] << 24);
                        motor_state_.target_velocity = target_vel_raw * 0.001; // Scale factor
                    }
                    
                    if (debug_enabled_) {
                        RCLCPP_INFO(this->get_logger(), "Received control: word=0x%04X, target_vel=%.3f", 
                                   motor_state_.control_word, motor_state_.target_velocity);
                    }
                }
                break;
                
            case 0x0B: // NMT Node Control
                // Handle NMT state changes
                break;
                
            default:
                if (debug_enabled_) {
                    RCLCPP_DEBUG(this->get_logger(), "Unhandled CAN frame: ID=0x%X, FC=%d", 
                               frame.can_id, function_code);
                }
                break;
        }
    }
    
    void updateMotorSimulation(double dt) {
        std::lock_guard<std::mutex> lock(motor_state_mutex_);
        
        if (motor_state_.enabled) {
            // Simple first-order response to target velocity
            double velocity_error = motor_state_.target_velocity - motor_state_.velocity;
            double velocity_gain = 5.0; // rad/s^2 per rad/s error
            
            motor_state_.velocity += velocity_gain * velocity_error * dt;
            
            // Apply velocity limits
            motor_state_.velocity = std::max(-max_velocity_, 
                                   std::min(max_velocity_, motor_state_.velocity));
            
            // Update position
            motor_state_.position += motor_state_.velocity * dt;
            
            // Update status word (simplified)
            motor_state_.status_word = 0x1637; // Operation enabled, target reached
        } else {
            motor_state_.velocity = 0.0;
            motor_state_.status_word = 0x0650; // Switch on disabled
        }
    }
    
    void sendCANopenStatus() {
        // Send TPDO1 (status word + actual velocity + position)
        CANFrame frame;
        frame.can_id = 0x180 + canopen_node_id_; // TPDO1 COB-ID
        frame.dlc = 8;
        frame.is_extended = false;
        frame.is_rtr = false;
        
        std::lock_guard<std::mutex> lock(motor_state_mutex_);
        
        // Status word (16-bit)
        frame.data[0] = motor_state_.status_word & 0xFF;
        frame.data[1] = (motor_state_.status_word >> 8) & 0xFF;
        
        // Actual velocity (32-bit, scaled)
        int32_t velocity_scaled = static_cast<int32_t>(motor_state_.velocity * 1000);
        frame.data[2] = velocity_scaled & 0xFF;
        frame.data[3] = (velocity_scaled >> 8) & 0xFF;
        frame.data[4] = (velocity_scaled >> 16) & 0xFF;
        frame.data[5] = (velocity_scaled >> 24) & 0xFF;
        
        // Position (16-bit, reduced precision)
        int16_t position_scaled = static_cast<int16_t>(motor_state_.position * 1000);
        frame.data[6] = position_scaled & 0xFF;
        frame.data[7] = (position_scaled >> 8) & 0xFF;
        
        if (!usb_can_driver_->sendFrame(frame)) {
            RCLCPP_WARN(this->get_logger(), "Failed to send status frame: %s", 
                       usb_can_driver_->getLastError().c_str());
        }
    }
    
    void publishROS2Status() {
        // Publish joint state
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = this->now();
        joint_msg.name.push_back("traction_joint");
        
        std::lock_guard<std::mutex> lock(motor_state_mutex_);
        joint_msg.position.push_back(motor_state_.position);
        joint_msg.velocity.push_back(motor_state_.velocity);
        joint_msg.effort.push_back(0.0); // No torque simulation
        
        joint_state_pub_->publish(joint_msg);
        
        // Publish status string
        auto status_msg = std_msgs::msg::String();
        status_msg.data = "Enabled: " + std::string(motor_state_.enabled ? "true" : "false") +
                         ", Velocity: " + std::to_string(motor_state_.velocity) +
                         ", Position: " + std::to_string(motor_state_.position);
        status_pub_->publish(status_msg);
    }
    
    void publishTurtleCommand() {
        // Convert motor velocity to turtle movement
        std::lock_guard<std::mutex> lock(motor_state_mutex_);
        
        auto twist_msg = geometry_msgs::msg::Twist();
        
        // Scale motor velocity to reasonable turtle speed
        double scale_factor = 0.2; // Adjust as needed
        twist_msg.linear.x = motor_state_.velocity * scale_factor;
        twist_msg.angular.z = 0.0; // No rotation for this simple test
        
        turtle_cmd_pub_->publish(twist_msg);
    }
    
    void stopSimulation() {
        running_ = false;
        
        if (can_receiver_thread_.joinable()) {
            can_receiver_thread_.join();
        }
        
        if (motor_simulation_thread_.joinable()) {
            motor_simulation_thread_.join();
        }
        
        if (usb_can_driver_) {
            usb_can_driver_->shutdown();
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<VirtualMotorSimulator>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
