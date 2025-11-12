// Copyright 2025 Andrea Efficace
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

#ifndef ROS2_WAVESHARE__VELOCITY_CONVERTER_NODE_HPP_
#define ROS2_WAVESHARE__VELOCITY_CONVERTER_NODE_HPP_

#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ros2_waveshare_msgs/msg/motor_command.hpp"
#include "ros2_waveshare_msgs/msg/motor_feedback.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "pluginlib/class_loader.hpp"
#include "ros2_waveshare/dynamic_model_interface.hpp"

namespace ros2_waveshare {

/**
 * @brief Node that converts velocity commands to motor commands using pluggable dynamic models.
 *
 * This node acts as the controller in a closed-loop velocity control system. It:
 * 1. Subscribes to cmd_vel (desired velocity from teleop or navigation)
 * 2. Subscribes to motor feedback (actual encoder measurements)
 * 3. Loads a dynamic model plugin (e.g., SingleMotorModel, DifferentialDriveModel)
 * 4. Runs control loop at high rate (default 100 Hz)
 * 5. Publishes motor commands based on plugin calculations
 *
 * The plugin architecture allows swapping between different kinematic models
 * (single motor, differential drive, Ackermann) without changing this node.
 *
 * Architecture:
 * ```
 *   cmd_vel (Twist) ──────┐
 *                         ├──> VelocityConverterNode ──> motor_command
 *   motor_feedback ───────┘         (plugin)
 * ```
 *
 * Control Flow:
 * 1. On cmd_vel: Store target velocity, reset timeout
 * 2. On motor_feedback: Store current state
 * 3. On timer (100 Hz):
 *    - Check cmd_vel timeout (safety)
 *    - Call plugin->update(target, current)
 *    - Call plugin->compute_control()
 *    - Publish motor command
 */
    class VelocityConverterNode : public rclcpp::Node {
        public:
            /**
             * @brief Construct a new Velocity Converter Node.
             *
             * Initializes:
             * - Parameters (plugin_name, control_rate, cmd_vel_timeout, motor_namespace)
             * - Pluginlib class loader for DynamicModelInterface
             * - Subscribers (cmd_vel, motor_feedback)
             * - Publishers (motor_command)
             * - Services (reload_plugin)
             * - Control loop timer
             */
            explicit VelocityConverterNode(
                const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

            /**
             * @brief Destructor - shutdown plugin gracefully.
             */
            ~VelocityConverterNode();

        private:
            // === Plugin Management ===

            /**
             * @brief Load the dynamic model plugin specified in parameters.
             *
             * @param plugin_name Fully qualified plugin class name (e.g., "ros2_waveshare::SingleMotorModel")
             * @return true if plugin loaded and initialized successfully
             * @return false if loading or initialization failed
             */
            bool load_plugin(const std::string& plugin_name);

            /**
             * @brief Unload the current plugin (calls shutdown()).
             */
            void unload_plugin();

            /**
             * @brief Service callback to reload the plugin.
             *
             * Useful for switching models at runtime or recovering from plugin errors.
             *
             * @param request Empty trigger request
             * @param response Success/failure message
             */
            void reload_plugin_callback(
                const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response);

            // === Subscribers ===

            /**
             * @brief Callback for cmd_vel topic (desired velocity).
             *
             * Stores target velocity and resets timeout counter.
             *
             * @param msg Twist message with linear.x and angular.z
             */
            void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

            /**
             * @brief Callback for motor feedback topic (encoder measurements).
             *
             * Stores current motor state for closed-loop control.
             *
             * @param msg MotorFeedback with position, velocity, current, etc.
             */
            void feedback_callback(const ros2_waveshare_msgs::msg::MotorFeedback::SharedPtr msg);

            // === Control Loop ===

            /**
             * @brief Main control loop timer callback.
             *
             * Executes at control_rate (default 100 Hz):
             * 1. Check cmd_vel timeout (safety stop if no recent commands)
             * 2. Update plugin with target and current velocities
             * 3. Compute control output
             * 4. Publish motor command
             * 5. Log state for debugging
             */
            void control_timer_callback();

            // === Parameters ===
            std::string plugin_name_; ///< Plugin class name to load
            double control_rate_;     ///< Control loop frequency (Hz)
            double cmd_vel_timeout_;  ///< Max age of cmd_vel before safety stop (seconds)
            std::string motor_namespace_; ///< Namespace for motor topics (e.g., "/motors/motor_1")
            uint8_t node_id_;         ///< CANopen node ID for motor command

            // === Pluginlib ===
            std::shared_ptr<pluginlib::ClassLoader<DynamicModelInterface> > plugin_loader_;
            std::shared_ptr<DynamicModelInterface> dynamic_model_;

            // === ROS2 Communication ===
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
            rclcpp::Subscription<ros2_waveshare_msgs::msg::MotorFeedback>::SharedPtr feedback_sub_;
            rclcpp::Publisher<ros2_waveshare_msgs::msg::MotorCommand>::SharedPtr motor_cmd_pub_;
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_plugin_srv_;
            rclcpp::TimerBase::SharedPtr control_timer_;

            // === State Variables ===
            geometry_msgs::msg::Twist::SharedPtr current_cmd_vel_;
            ros2_waveshare_msgs::msg::MotorFeedback::SharedPtr current_feedback_;
            rclcpp::Time last_cmd_vel_time_;
            rclcpp::Time last_control_time_;
            bool feedback_received_;  ///< Has first feedback arrived?
            bool plugin_initialized_; ///< Is plugin loaded and ready?

            // === Statistics (for debugging) ===
            size_t control_loop_count_;
            size_t cmd_vel_count_;
            size_t feedback_count_;
    };

}  // namespace ros2_waveshare

#endif  // ROS2_WAVESHARE__VELOCITY_CONVERTER_NODE_HPP_
