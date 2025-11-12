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

#ifndef ROS2_WAVESHARE__DYNAMIC_MODEL_INTERFACE_HPP_
#define ROS2_WAVESHARE__DYNAMIC_MODEL_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ros2_waveshare_msgs/msg/motor_feedback.hpp>
#include <ros2_waveshare_msgs/msg/motor_command.hpp>
#include <string>

namespace ros2_waveshare {

/**
 * @brief Pure virtual interface for dynamic model plugins
 *
 * This interface defines the contract for all dynamic model plugins used
 * by the velocity converter node. Plugins implement different kinematic
 * and dynamic models (e.g., single motor, differential drive, Ackermann).
 *
 * The plugin architecture follows the ros2_control pattern using pluginlib
 * for runtime loading and switching between different models.
 *
 * Plugin Lifecycle:
 * 1. Construction (by pluginlib)
 * 2. initialize() - Load parameters, allocate resources
 * 3. update() - Called at control rate with new data
 * 4. compute_control() - Calculate motor commands
 * 5. reset() - Clear internal state (optional, on user request)
 * 6. shutdown() - Prepare for safe stop
 * 7. Destruction
 *
 * Example Plugins:
 * - SingleMotorModel: PID velocity control for one motor
 * - DifferentialDriveModel: Two-wheel differential drive kinematics
 * - AckermannModel: Car-like steering with Ackermann geometry
 */
    class DynamicModelInterface {
        public:
            /**
             * @brief Virtual destructor
             */
            virtual ~DynamicModelInterface() = default;

            /**
             * @brief Initialize the dynamic model plugin
             *
             * Called once after plugin construction. Load parameters, initialize
             * state variables, and allocate any required resources.
             *
             * @param node ROS2 node for parameter access and logging
             * @param param_namespace Parameter namespace for this plugin (e.g., "single_motor_model")
             * @return true if initialization successful, false otherwise
             */
            virtual bool initialize(
                rclcpp::Node::SharedPtr node,
                const std::string& param_namespace) = 0;

            /**
             * @brief Update model state with new command and feedback
             *
             * Called at the control rate (typically 100 Hz) with the latest target
             * velocity command and motor feedback. Plugin should update internal
             * state but not yet compute output.
             *
             * @param cmd_vel Target velocity command from teleop/navigation
             * @param feedback Current motor state (position, velocity, torque, etc.)
             * @param dt Time since last update (seconds)
             */
            virtual void update(
                const geometry_msgs::msg::Twist& cmd_vel,
                const ros2_waveshare_msgs::msg::MotorFeedback& feedback,
                double dt) = 0;

            /**
             * @brief Compute motor command based on current state
             *
             * Called after update() to calculate the motor command to send to
             * the motor driver. This is where control algorithms (PID, feedforward,
             * etc.) are applied.
             *
             * @return Motor command message with target velocity/position/torque
             */
            virtual ros2_waveshare_msgs::msg::MotorCommand compute_control() = 0;

            /**
             * @brief Reset internal state
             *
             * Clears integrators, error history, and other internal state.
             * Used when restarting control or recovering from faults.
             * Does not change configuration/parameters.
             */
            virtual void reset() = 0;

            /**
             * @brief Prepare for safe shutdown
             *
             * Called when system is shutting down. Returns a final motor command
             * that safely stops the motor (typically zero velocity).
             *
             * @return Final motor command for safe stop
             */
            virtual ros2_waveshare_msgs::msg::MotorCommand shutdown() = 0;

            /**
             * @brief Get current model state as string (for debugging)
             *
             * Returns human-readable representation of internal state.
             * Useful for diagnostics and tuning.
             *
             * @return String describing current state
             */
            virtual std::string get_state_string() const = 0;
    };

}  // namespace ros2_waveshare

#endif  // ROS2_WAVESHARE__DYNAMIC_MODEL_INTERFACE_HPP_
