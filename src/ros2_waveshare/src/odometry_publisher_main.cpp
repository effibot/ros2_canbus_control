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

#include "ros2_waveshare/odometry_publisher_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);

    try {
        // Create odometry publisher node
        auto options = rclcpp::NodeOptions();
        auto node = std::make_shared<ros2_waveshare::OdometryPublisherNode>(options);

        // Spin node
        rclcpp::spin(node);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("odometry_publisher_main"),
            "Exception in odometry publisher node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
