// Copyright 2025 Andrea Efficace (andrea.efficace1@gmail.com)
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

#include "ros2_waveshare/motor_instance.hpp"

namespace ros2_waveshare {

    MotorInstance::MotorInstance(uint8_t node_id, const std::string& name, const std::string& type)
        : node_id_(node_id),
        name_(name),
        type_(type) {
        // Initialize timestamps to current time
        last_tpdo1_time_ = std::chrono::steady_clock::now();
        last_tpdo2_time_ = std::chrono::steady_clock::now();

        // Initialize feedback message header
        latest_feedback_.node_id = node_id;
        latest_feedback_.motor_name = name;
    }

}  // namespace ros2_waveshare
