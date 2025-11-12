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

#include "rclcpp/rclcpp.hpp"
#include "ros2_waveshare/velocity_converter_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<ros2_waveshare::VelocityConverterNode>();
        rclcpp::spin(node);
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(rclcpp::get_logger("velocity_converter_main"),
            "Exception in velocity_converter_node: %s", ex.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
