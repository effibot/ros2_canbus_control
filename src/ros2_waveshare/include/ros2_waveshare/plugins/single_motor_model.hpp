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

#ifndef ROS2_WAVESHARE__PLUGINS__SINGLE_MOTOR_MODEL_HPP_
#define ROS2_WAVESHARE__PLUGINS__SINGLE_MOTOR_MODEL_HPP_

#include "ros2_waveshare/dynamic_model_interface.hpp"
#include <rclcpp/rclcpp.hpp>

namespace ros2_waveshare {

/**
 * @brief Single motor velocity control using PID
 *
 * Implements closed-loop velocity control for a single motor using
 * a PID controller. Converts linear velocity commands to angular
 * motor velocity using wheel radius.
 *
 * Control Loop:
 * 1. Receive target linear velocity (m/s) from cmd_vel
 * 2. Convert to target angular velocity (rad/s): ω_target = v / r
 * 3. Read actual angular velocity from encoder feedback
 * 4. PID controller computes correction based on error
 * 5. Output motor command with target velocity
 *
 * Features:
 * - PID control with tunable gains (kp, ki, kd)
 * - Anti-windup for integral term
 * - Acceleration limiting for smooth motion
 * - Velocity and acceleration limits
 * - Automatic unit conversion (linear ↔ angular)
 *
 * Parameters:
 * - kp: Proportional gain
 * - ki: Integral gain
 * - kd: Derivative gain
 * - max_velocity: Maximum linear velocity (m/s)
 * - max_acceleration: Maximum acceleration (m/s²)
 * - max_integral: Anti-windup limit for integral term
 * - wheel_radius: Wheel radius for unit conversion (m)
 * - gear_ratio: Gear reduction ratio (motor/wheel)
 */
    class SingleMotorModel : public DynamicModelInterface {
        public:
            /**
             * @brief Constructor
             */
            SingleMotorModel();

            /**
             * @brief Destructor
             */
            ~SingleMotorModel() override = default;

            /**
             * @brief Initialize plugin with parameters
             *
             * @param node ROS2 node for parameter access
             * @param param_namespace Parameter namespace (e.g., "single_motor_model")
             * @return true if initialization successful
             */
            bool initialize(
                rclcpp::Node::SharedPtr node,
                const std::string& param_namespace) override;

            /**
             * @brief Update with new target and feedback
             *
             * @param cmd_vel Target velocity command (Twist message)
             * @param feedback Current motor state from encoder
             * @param dt Time since last update (seconds)
             */
            void update(
                const geometry_msgs::msg::Twist& cmd_vel,
                const ros2_waveshare_msgs::msg::MotorFeedback& feedback,
                double dt) override;

            /**
             * @brief Compute PID control output
             *
             * @return Motor command with target velocity
             */
            ros2_waveshare_msgs::msg::MotorCommand compute_control() override;

            /**
             * @brief Reset PID state (clear integral, errors)
             */
            void reset() override;

            /**
             * @brief Prepare for shutdown
             *
             * @return Zero velocity command for safe stop
             */
            ros2_waveshare_msgs::msg::MotorCommand shutdown() override;

            /**
             * @brief Get current state as string
             *
             * @return String with target, current, error, integral, etc.
             */
            std::string get_state_string() const override;

        private:
            /**
             * @brief Clamp value between min and max
             */
            double clamp(double value, double min_val, double max_val) const;

            // ROS2 node (for logging)
            rclcpp::Node::SharedPtr node_;

            // PID Controller State
            double target_velocity_rad_s_; ///< Target angular velocity (rad/s)
            double current_velocity_rad_s_; ///< Current angular velocity from encoder (rad/s)
            double error_;            ///< Current velocity error (rad/s)
            double integral_error_;   ///< Accumulated integral error
            double previous_error_;   ///< Previous error for derivative calculation
            double previous_output_;  ///< Previous control output (for acceleration limiting)

            // PID Gains
            double kp_;               ///< Proportional gain
            double ki_;               ///< Integral gain
            double kd_;               ///< Derivative gain

            // Limits
            double max_velocity_;     ///< Maximum linear velocity (m/s)
            double max_acceleration_; ///< Maximum acceleration (m/s²)
            double max_integral_;     ///< Anti-windup limit for integral term

            // Physical Parameters
            double wheel_radius_;     ///< Wheel radius (m)
            double gear_ratio_;       ///< Gear reduction ratio (motor:wheel)

            // State flags
            bool initialized_;        ///< Plugin initialized flag
    };

}  // namespace ros2_waveshare

#endif  // ROS2_WAVESHARE__PLUGINS__SINGLE_MOTOR_MODEL_HPP_
