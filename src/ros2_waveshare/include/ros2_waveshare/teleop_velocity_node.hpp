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

#ifndef ROS2_WAVESHARE__TELEOP_VELOCITY_NODE_HPP_
#define ROS2_WAVESHARE__TELEOP_VELOCITY_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <atomic>
#include <thread>

namespace ros2_waveshare {

/**
 * @brief Teleop node state machine states
 *
 * State transitions follow the diagram in teleop_state_machine.mmd
 */
    enum class TeleopState {
        INITIALIZING, ///< Node starting up, configuring parameters
        IDLE,     ///< Waiting for user input, velocity = 0
        ACCELERATING, ///< Increasing velocity (positive or negative)
        DECELERATING, ///< Decreasing velocity magnitude
        CRUISING, ///< Maintaining constant velocity
        STOPPING, ///< Ramping to zero velocity
        SHUTDOWN  ///< Safe shutdown sequence in progress
    };

/**
 * @brief Simple keyboard teleoperation node for velocity control
 *
 * Provides keyboard interface for controlling robot velocity:
 * - ↑ Arrow: Increase velocity by accel_step
 * - ↓ Arrow: Decrease velocity by accel_step
 * - '0' Key: Ramp to zero velocity
 * - ESC Key: Shutdown (send zero, exit)
 *
 * Features:
 * - Smooth acceleration/deceleration
 * - Velocity limits (±max_velocity)
 * - Safe shutdown sequence
 * - Real-time keyboard input (non-blocking)
 */
    class TeleopVelocityNode : public rclcpp::Node {
        public:
            /**
             * @brief Constructor
             *
             * Initializes ROS2 node, declares parameters, sets up publisher,
             * configures terminal for non-blocking keyboard input
             */
            explicit TeleopVelocityNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

            /**
             * @brief Destructor
             *
             * Restores terminal settings, stops keyboard thread
             */
            ~TeleopVelocityNode();

            /**
             * @brief Start the teleop control loop
             *
             * Spawns keyboard input thread and starts publishing timer
             */
            void start();

            /**
             * @brief Stop the teleop control loop
             *
             * Initiates shutdown sequence, stops threads
             */
            void stop();

        private:
            /**
             * @brief Publishing timer callback
             *
             * Publishes cmd_vel at configured rate (default 20 Hz)
             */
            void publish_callback();

            /**
             * @brief Keyboard input processing thread
             *
             * Continuously reads keyboard input and updates target velocity
             * Runs in separate thread to avoid blocking main loop
             */
            void keyboard_thread();

            /**
             * @brief Read single character from keyboard (non-blocking)
             *
             * @return Character code, or -1 if no input available
             */
            int get_key();

            /**
             * @brief Process keyboard input and update target velocity
             *
             * @param key Character code from keyboard
             * @return true if should continue, false to shutdown
             */
            bool process_key(int key);

            /**
             * @brief Configure terminal for non-blocking input
             *
             * Disables echo and canonical mode for raw keyboard access
             */
            void configure_terminal();

            /**
             * @brief Restore original terminal settings
             *
             * Called on shutdown to return terminal to normal mode
             */
            void restore_terminal();

            /**
             * @brief Update state machine based on velocity changes
             *
             * Implements state transitions from teleop_state_machine.mmd
             */
            void update_state();

            /**
             * @brief Get human-readable state name
             *
             * @param state Current teleop state
             * @return String representation of state
             */
            std::string get_state_name(TeleopState state) const;

            /**
             * @brief Print usage instructions to console
             */
            void print_usage();

            // ROS2 interfaces
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
            rclcpp::TimerBase::SharedPtr publish_timer_;

            // State management
            TeleopState current_state_;
            TeleopState previous_state_;
            std::atomic<double> target_velocity_; ///< Target linear velocity (m/s)
            std::atomic<double> current_velocity_; ///< Current published velocity (m/s)

            // Parameters
            double max_velocity_; ///< Maximum velocity magnitude (m/s)
            double accel_step_; ///< Velocity change per keypress (m/s)
            double publish_rate_; ///< Publishing frequency (Hz)

            // Terminal control
            struct termios original_terminal_settings_;
            bool terminal_configured_;

            // Threading
            std::thread keyboard_thread_;
            std::atomic<bool> running_;
            std::atomic<bool> shutdown_requested_;

            // Constants for keyboard codes
            static constexpr int KEY_ARROW_UP = 65; ///< ANSI up arrow
            static constexpr int KEY_ARROW_DOWN = 66; ///< ANSI down arrow
            static constexpr int KEY_ZERO = '0'; ///< Zero key
            static constexpr int KEY_ESC = 27; ///< Escape key
            static constexpr int KEY_BRACKET = 91; ///< '[' for arrow keys
    };

}  // namespace ros2_waveshare

#endif  // ROS2_WAVESHARE__TELEOP_VELOCITY_NODE_HPP_
