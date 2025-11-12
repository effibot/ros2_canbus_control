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
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <csignal>

// Global pointer for signal handler
std::shared_ptr<ros2_waveshare::TeleopVelocityNode> g_node = nullptr;

/**
 * @brief Signal handler for graceful shutdown
 *
 * Handles Ctrl+C (SIGINT) and ensures safe shutdown sequence
 */
void signal_handler(int signum) {
    (void)signum; // Unused parameter

    if (g_node) {
        RCLCPP_INFO(g_node->get_logger(), "Interrupt received - shutting down gracefully");
        g_node->stop();
        rclcpp::shutdown();
    }
}

int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);

    try {
        // Create teleop node
        auto options = rclcpp::NodeOptions();
        g_node = std::make_shared<ros2_waveshare::TeleopVelocityNode>(options);

        // Register signal handler
        std::signal(SIGINT, signal_handler);

        // Start teleop (keyboard thread and publishing)
        g_node->start();

        // Spin node
        rclcpp::spin(g_node);

        // Clean shutdown
        g_node->stop();
        g_node.reset();

    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("teleop_velocity_main"),
            "Exception in teleop node: %s", e.what());
        if (g_node) {
            g_node->stop();
        }
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
