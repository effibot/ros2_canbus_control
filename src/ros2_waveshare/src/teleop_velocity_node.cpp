// Copyright 2025 Effibot
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros2_waveshare/teleop_velocity_node.hpp"
#include <unistd.h>
#include <fcntl.h>
#include <cmath>
#include <iostream>
#include <iomanip>

namespace ros2_waveshare {

    TeleopVelocityNode::TeleopVelocityNode(const rclcpp::NodeOptions& options)
        : Node("teleop_velocity", options),
        current_state_(TeleopState::INITIALIZING),
        previous_state_(TeleopState::INITIALIZING),
        target_velocity_(0.0),
        current_velocity_(0.0),
        terminal_configured_(false),
        running_(false),
        shutdown_requested_(false) {
        // Declare and get parameters
        this->declare_parameter("max_velocity", 1.0);
        this->declare_parameter("accel_step", 0.1);
        this->declare_parameter("publish_rate", 20.0);

        max_velocity_ = this->get_parameter("max_velocity").as_double();
        accel_step_ = this->get_parameter("accel_step").as_double();
        publish_rate_ = this->get_parameter("publish_rate").as_double();

        // Validate parameters
        if (max_velocity_ <= 0.0) {
            RCLCPP_ERROR(this->get_logger(), "max_velocity must be positive, got: %.3f",
                max_velocity_);
            throw std::runtime_error("Invalid max_velocity parameter");
        }
        if (accel_step_ <= 0.0) {
            RCLCPP_ERROR(this->get_logger(), "accel_step must be positive, got: %.3f", accel_step_);
            throw std::runtime_error("Invalid accel_step parameter");
        }
        if (publish_rate_ <= 0.0) {
            RCLCPP_ERROR(this->get_logger(), "publish_rate must be positive, got: %.3f",
                publish_rate_);
            throw std::runtime_error("Invalid publish_rate parameter");
        }

        RCLCPP_INFO(this->get_logger(),
            "Teleop parameters: max_vel=%.2f m/s, accel_step=%.2f m/s, rate=%.1f Hz",
            max_velocity_, accel_step_, publish_rate_);

        // Create cmd_vel publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Create publishing timer
        auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);
        publish_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
            std::bind(&TeleopVelocityNode::publish_callback, this));

        RCLCPP_INFO(this->get_logger(), "TeleopVelocityNode initialized");
        current_state_ = TeleopState::IDLE;
    }

    TeleopVelocityNode::~TeleopVelocityNode() {
        stop();
        restore_terminal();
    }

    void TeleopVelocityNode::start() {
        if (running_) {
            RCLCPP_WARN(this->get_logger(), "Teleop already running");
            return;
        }

        // Configure terminal for raw input
        configure_terminal();

        // Print usage instructions
        print_usage();

        // Start keyboard thread
        running_ = true;
        keyboard_thread_ = std::thread(&TeleopVelocityNode::keyboard_thread, this);

        RCLCPP_INFO(this->get_logger(), "Teleop started - waiting for keyboard input");
    }

    void TeleopVelocityNode::stop() {
        if (!running_) {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Stopping teleop...");

        // Signal shutdown
        shutdown_requested_ = true;
        current_state_ = TeleopState::SHUTDOWN;

        // Stop keyboard thread
        running_ = false;
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }

        // Send final zero velocity
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 0.0;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = 0.0;
        cmd_vel_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Teleop stopped - final zero velocity sent");
    }

    void TeleopVelocityNode::publish_callback() {
        // Update state machine
        update_state();

        // Create and publish Twist message
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = current_velocity_;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = 0.0;

        cmd_vel_pub_->publish(msg);

        // Log state changes
        if (current_state_ != previous_state_) {
            RCLCPP_INFO(this->get_logger(), "State: %s -> %s | Velocity: %.3f m/s",
                get_state_name(previous_state_).c_str(),
                get_state_name(current_state_).c_str(),
                current_velocity_.load());
            previous_state_ = current_state_;
        }
    }

    void TeleopVelocityNode::keyboard_thread() {
        while (running_) {
            int key = get_key();
            if (key != -1) {
                if (!process_key(key)) {
                    // Shutdown requested
                    break;
                }
            }
            // Small sleep to avoid busy waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    int TeleopVelocityNode::get_key() {
        // Set stdin to non-blocking
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

        int key = getchar();

        // Restore blocking mode
        fcntl(STDIN_FILENO, F_SETFL, flags);

        return key;
    }

    bool TeleopVelocityNode::process_key(int key) {
        double new_target = target_velocity_;

        // Handle escape sequences for arrow keys
        if (key == KEY_ESC) {
            // Could be ESC key or start of escape sequence
            int next = get_key();
            if (next == KEY_BRACKET) {
                // Arrow key sequence: ESC [ A/B/C/D
                int arrow = get_key();
                if (arrow == KEY_ARROW_UP) {
                    // Up arrow - increase velocity
                    new_target = std::min(target_velocity_.load() + accel_step_, max_velocity_);
                    RCLCPP_INFO(this->get_logger(), "↑ Accelerating to %.3f m/s", new_target);
                } else if (arrow == KEY_ARROW_DOWN) {
                    // Down arrow - decrease velocity
                    new_target = std::max(target_velocity_.load() - accel_step_, -max_velocity_);
                    RCLCPP_INFO(this->get_logger(), "↓ Decelerating to %.3f m/s", new_target);
                }
            } else if (next == -1) {
                // Pure ESC - shutdown
                RCLCPP_INFO(this->get_logger(), "ESC pressed - initiating shutdown");
                target_velocity_ = 0.0;
                shutdown_requested_ = true;
                return false;
            }
        } else if (key == KEY_ZERO) {
            // '0' key - stop (ramp to zero)
            new_target = 0.0;
            RCLCPP_INFO(this->get_logger(), "0 pressed - stopping");
        }

        // Update target velocity
        target_velocity_ = new_target;

        // Smoothly ramp current velocity towards target
        double current = current_velocity_.load();
        double diff = new_target - current;
        double max_change = accel_step_; // Maximum change per update cycle

        if (std::abs(diff) < max_change) {
            current_velocity_ = new_target;
        } else {
            current_velocity_ = current + (diff >
                0 ? max_change : -max_change) * (publish_rate_ / 20.0);
        }

        return true;
    }

    void TeleopVelocityNode::configure_terminal() {
        // Get current terminal settings
        if (tcgetattr(STDIN_FILENO, &original_terminal_settings_) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get terminal attributes");
            return;
        }

        // Copy to new settings
        struct termios new_settings = original_terminal_settings_;

        // Disable canonical mode (line buffering) and echo
        new_settings.c_lflag &= ~(ICANON | ECHO);

        // Set minimum characters to read
        new_settings.c_cc[VMIN] = 0;
        new_settings.c_cc[VTIME] = 0;

        // Apply new settings
        if (tcsetattr(STDIN_FILENO, TCSANOW, &new_settings) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set terminal attributes");
            return;
        }

        terminal_configured_ = true;
        RCLCPP_DEBUG(this->get_logger(), "Terminal configured for raw input");
    }

    void TeleopVelocityNode::restore_terminal() {
        if (terminal_configured_) {
            if (tcsetattr(STDIN_FILENO, TCSANOW, &original_terminal_settings_) < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to restore terminal attributes");
            } else {
                RCLCPP_DEBUG(this->get_logger(), "Terminal settings restored");
            }
            terminal_configured_ = false;
        }
    }

    void TeleopVelocityNode::update_state() {
        double current = current_velocity_.load();
        double target = target_velocity_.load();

        const double epsilon = 0.001; // Threshold for "zero" velocity

        if (shutdown_requested_) {
            current_state_ = TeleopState::SHUTDOWN;
        } else if (std::abs(current) < epsilon && std::abs(target) < epsilon) {
            current_state_ = TeleopState::IDLE;
        } else if (std::abs(target) < epsilon && std::abs(current) > epsilon) {
            current_state_ = TeleopState::STOPPING;
        } else if (std::abs(target - current) < epsilon) {
            current_state_ = TeleopState::CRUISING;
        } else if (std::abs(target) > std::abs(current)) {
            current_state_ = TeleopState::ACCELERATING;
        } else {
            current_state_ = TeleopState::DECELERATING;
        }
    }

    std::string TeleopVelocityNode::get_state_name(TeleopState state) const {
        switch (state) {
        case TeleopState::INITIALIZING: return "INITIALIZING";
        case TeleopState::IDLE: return "IDLE";
        case TeleopState::ACCELERATING: return "ACCELERATING";
        case TeleopState::DECELERATING: return "DECELERATING";
        case TeleopState::CRUISING: return "CRUISING";
        case TeleopState::STOPPING: return "STOPPING";
        case TeleopState::SHUTDOWN: return "SHUTDOWN";
        default: return "UNKNOWN";
        }
    }

    void TeleopVelocityNode::print_usage() {
        std::cout << "\n";
        std::cout << "╔════════════════════════════════════════════════════════════╗\n";
        std::cout << "║         ROS2 Waveshare - Teleop Velocity Control          ║\n";
        std::cout << "╠════════════════════════════════════════════════════════════╣\n";
        std::cout << "║                                                            ║\n";
        std::cout << "║  Keyboard Controls:                                        ║\n";
        std::cout << "║    ↑  (Up Arrow)    : Increase velocity by " << std::fixed <<
            std::setprecision(2)
                  << std::setw(5) << accel_step_ << " m/s       ║\n";
        std::cout << "║    ↓  (Down Arrow)  : Decrease velocity by " << std::fixed <<
            std::setprecision(2)
                  << std::setw(5) << accel_step_ << " m/s       ║\n";
        std::cout << "║    0  (Zero Key)    : Ramp to zero velocity                ║\n";
        std::cout << "║    ESC              : Shutdown (safe stop)                 ║\n";
        std::cout << "║                                                            ║\n";
        std::cout << "║  Parameters:                                               ║\n";
        std::cout << "║    Max Velocity     : ±" << std::fixed << std::setprecision(2)
                  << std::setw(5) << max_velocity_ << " m/s                      ║\n";
        std::cout << "║    Accel Step       : " << std::fixed << std::setprecision(2)
                  << std::setw(5) << accel_step_ << " m/s                         ║\n";
        std::cout << "║    Publish Rate     : " << std::fixed << std::setprecision(1)
                  << std::setw(5) << publish_rate_ << " Hz                          ║\n";
        std::cout << "║                                                            ║\n";
        std::cout << "║  Topic:                                                    ║\n";
        std::cout << "║    /cmd_vel (geometry_msgs/msg/Twist)                      ║\n";
        std::cout << "║                                                            ║\n";
        std::cout << "╚════════════════════════════════════════════════════════════╝\n";
        std::cout << "\nReady for input...\n\n";
    }

}  // namespace ros2_waveshare
