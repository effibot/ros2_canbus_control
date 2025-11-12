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

/**
 * @file motor_driver_handlers.cpp
 * @brief ROS2 callback handler implementations for MotorDriverNode
 */

#include "ros2_waveshare/motor_driver_node.hpp"
#include "ros2_waveshare/motor_driver_action_helpers.hpp"
#include <canopen/cia402_constants.hpp>

namespace ros2_waveshare {

// =============================================================================
// Service Handlers
// =============================================================================

    void MotorDriverNode::handle_sdo_read(
        uint8_t node_id,
        std::shared_ptr<SDORead::Request> request,
        std::shared_ptr<SDORead::Response> response) {
        auto motor = get_motor(node_id);
        if (!motor) {
            response->success = false;
            response->message = "Motor with node_id " + std::to_string(node_id) + " not found";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        auto sdo_client = motor->get_sdo_client();
        if (!sdo_client) {
            response->success = false;
            response->message = "SDO client not initialized for motor " + motor->get_name();
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        try {
            std::vector<uint8_t> data;

            // Use object name if provided, otherwise use index/subindex
            if (!request->object_name.empty()) {
                data = sdo_client->read_object(request->object_name);
                RCLCPP_INFO(this->get_logger(),
                    "SDO read '%s' from node %d: %zu bytes",
                    request->object_name.c_str(), node_id, data.size());
            } else {
                // For direct index/subindex access, we need to construct an SDO read manually
                // This is not directly supported by the current API, so use object name
                response->success = false;
                response->message =
                    "Direct index/subindex read not supported. Use object_name instead.";
                RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
                return;
            }

            response->success = true;
            response->data = data;
            response->data_length = data.size();

            // Parse data using ObjectDictionary helpers based on size
            auto& dict = motor->get_dictionary();

            if (data.size() == 1) {
                response->value_u8 = dict.from_raw<uint8_t>(data);
                response->value_i8 = dict.from_raw<int8_t>(data);
                response->detected_type = "uint8";
            } else if (data.size() == 2) {
                response->value_u16 = dict.from_raw<uint16_t>(data);
                response->value_i16 = dict.from_raw<int16_t>(data);
                response->detected_type = "uint16";
            } else if (data.size() == 4) {
                response->value_u32 = dict.from_raw<uint32_t>(data);
                response->value_i32 = dict.from_raw<int32_t>(data);
                response->detected_type = "uint32";
            } else {
                response->detected_type = "raw_bytes";
            }

            response->message = "SDO read successful";

        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("SDO read failed: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    void MotorDriverNode::handle_sdo_write(
        uint8_t node_id,
        std::shared_ptr<SDOWrite::Request> request,
        std::shared_ptr<SDOWrite::Response> response) {
        auto motor = get_motor(node_id);
        if (!motor) {
            response->success = false;
            response->message = "Motor with node_id " + std::to_string(node_id) + " not found";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        auto sdo_client = motor->get_sdo_client();
        if (!sdo_client) {
            response->success = false;
            response->message = "SDO client not initialized for motor " + motor->get_name();
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        try {
            std::vector<uint8_t> data_to_write;
            auto& dict = motor->get_dictionary();

            // Determine data to write based on value_type or infer from non-zero values
            if (!request->value_type.empty()) {
                // Explicit type specified - use ObjectDictionary helpers
                if (request->value_type == "uint8") {
                    data_to_write = dict.to_raw(request->value_u8);
                } else if (request->value_type == "uint16") {
                    data_to_write = dict.to_raw(request->value_u16);
                } else if (request->value_type == "uint32") {
                    data_to_write = dict.to_raw(request->value_u32);
                } else if (request->value_type == "int8") {
                    data_to_write = dict.to_raw(request->value_i8);
                } else if (request->value_type == "int16") {
                    data_to_write = dict.to_raw(request->value_i16);
                } else if (request->value_type == "int32") {
                    data_to_write = dict.to_raw(request->value_i32);
                } else if (request->value_type == "bytes") {
                    data_to_write = request->value_bytes;
                } else {
                    response->success = false;
                    response->message = "Unknown value_type: " + request->value_type;
                    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
                    return;
                }
            } else if (!request->value_bytes.empty()) {
                // Raw bytes provided
                data_to_write = request->value_bytes;
            } else {
                // Infer type from first non-zero value (prioritize larger types)
                if (request->value_u32 != 0 || request->value_i32 != 0) {
                    data_to_write = (request->value_u32 != 0) ?
                        dict.to_raw(request->value_u32) :
                        dict.to_raw(request->value_i32);
                } else if (request->value_u16 != 0 || request->value_i16 != 0) {
                    data_to_write = (request->value_u16 != 0) ?
                        dict.to_raw(request->value_u16) :
                        dict.to_raw(request->value_i16);
                } else if (request->value_u8 != 0 || request->value_i8 != 0) {
                    data_to_write = (request->value_u8 != 0) ?
                        dict.to_raw(request->value_u8) :
                        dict.to_raw(request->value_i8);
                } else {
                    response->success = false;
                    response->message = "No value provided for SDO write";
                    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
                    return;
                }
            }

            // Perform the write using object name
            if (!request->object_name.empty()) {
                bool write_success = sdo_client->write_object(
                    request->object_name,
                    data_to_write,
                    std::chrono::milliseconds(1000)
                );

                if (write_success) {
                    response->success = true;
                    response->message = "SDO write successful";
                    response->verified = false;  // Not implemented yet

                    RCLCPP_INFO(this->get_logger(),
                        "SDO write '%s' to node %d: %zu bytes written",
                        request->object_name.c_str(), node_id, data_to_write.size());
                } else {
                    response->success = false;
                    response->message = "SDO write failed (timeout or NAK)";
                    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
                }
            } else {
                // Direct index/subindex not supported yet
                response->success = false;
                response->message =
                    "Direct index/subindex write not supported. Use object_name instead.";
                RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            }

        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("SDO write failed: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    void MotorDriverNode::handle_set_operation_mode(
        uint8_t node_id,
        std::shared_ptr<SetOperationMode::Request> request,
        std::shared_ptr<SetOperationMode::Response> response) {
        auto motor = get_motor(node_id);
        if (!motor) {
            response->success = false;
            response->message = "Motor with node_id " + std::to_string(node_id) + " not found";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        auto sdo_client = motor->get_sdo_client();
        if (!sdo_client) {
            response->success = false;
            response->message = "SDO client not initialized for motor " + motor->get_name();
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        try {
            // First, read the current mode from Modes of Operation Display (0x6061)
            std::vector<uint8_t> current_mode_data = sdo_client->read_object(
                "modes_of_operation_display",
                std::chrono::milliseconds(1000)
            );

            int8_t previous_mode = 0;
            if (!current_mode_data.empty()) {
                previous_mode = static_cast<int8_t>(current_mode_data[0]);
            }
            response->previous_mode = previous_mode;

            // Write the new mode to Modes of Operation (0x6060)
            std::vector<uint8_t> mode_data;
            mode_data.push_back(static_cast<uint8_t>(request->operation_mode));

            bool write_success = sdo_client->write_object(
                "modes_of_operation",
                mode_data,
                std::chrono::milliseconds(1000)
            );

            if (!write_success) {
                response->success = false;
                response->message = "Failed to write operation mode (timeout or NAK)";
                response->current_mode = previous_mode;
                RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
                return;
            }

            // Verify the mode was set by reading Modes of Operation Display
            std::vector<uint8_t> verify_mode_data = sdo_client->read_object(
                "modes_of_operation_display",
                std::chrono::milliseconds(1000)
            );

            int8_t actual_mode = 0;
            if (!verify_mode_data.empty()) {
                actual_mode = static_cast<int8_t>(verify_mode_data[0]);
            }
            response->current_mode = actual_mode;

            // Check if mode was successfully set
            if (actual_mode == request->operation_mode) {
                response->success = true;
                response->message = "Operation mode set and verified successfully";

                RCLCPP_INFO(this->get_logger(),
                    "Motor %d operation mode changed: %d -> %d (%s)",
                    node_id, previous_mode, actual_mode,
                    canopen::cia402::get_mode_description(actual_mode));
            } else {
                response->success = false;
                response->message = "Mode verification failed: requested " +
                    std::to_string(request->operation_mode) +
                    " (" + canopen::cia402::get_mode_description(request->operation_mode) + ")" +
                    " but got " + std::to_string(actual_mode) +
                    " (" + canopen::cia402::get_mode_description(actual_mode) + ")";
                RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            }

        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Set operation mode failed: ") + e.what();
            response->current_mode = response->previous_mode;
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    void MotorDriverNode::handle_get_motor_info(
        uint8_t node_id,
        std::shared_ptr<GetMotorInfo::Request> request,
        std::shared_ptr<GetMotorInfo::Response> response) {
        auto motor = get_motor(node_id);
        if (!motor) {
            response->success = false;
            response->message = "Motor with node_id " + std::to_string(node_id) + " not found";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        auto sdo_client = motor->get_sdo_client();
        if (!sdo_client) {
            response->success = false;
            response->message = "SDO client not initialized for motor " + motor->get_name();
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        try {
            // Basic motor identification
            response->motor_name = motor->get_name();
            auto& dict = motor->get_dictionary();

            // Read device type (0x1000)
            try {
                auto device_type_data = sdo_client->read_object(
                    "device_type",
                    std::chrono::milliseconds(500)
                );
                response->device_type = dict.from_raw<uint32_t>(device_type_data);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to read device type: %s", e.what());
            }

            // Read statusword (0x6041)
            try {
                auto statusword_data = sdo_client->read_object(
                    "statusword",
                    std::chrono::milliseconds(500)
                );
                response->statusword = dict.from_raw<uint16_t>(statusword_data);

                // Decode CIA 402 state from statusword using waveshare_cpp helper
                auto state = canopen::cia402::decode_statusword(response->statusword);
                response->state = canopen::cia402::get_state_description(state);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to read statusword: %s", e.what());
            }

            // Read error register (0x1001)
            try {
                auto error_reg_data = sdo_client->read_object(
                    "error_register",
                    std::chrono::milliseconds(500)
                );
                response->error_register = dict.from_raw<uint8_t>(error_reg_data);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to read error register: %s", e.what());
            }

            // Read vendor ID (0x1018.01)
            try {
                auto vendor_data = sdo_client->read_object(
                    "identity_vendor_id",
                    std::chrono::milliseconds(500)
                );
                response->vendor_id = dict.from_raw<uint32_t>(vendor_data);
            } catch (const std::exception& e) {
                RCLCPP_DEBUG(this->get_logger(), "Failed to read vendor ID: %s", e.what());
            }

            // Read product code (0x1018.02)
            try {
                auto product_data = sdo_client->read_object(
                    "identity_product_code",
                    std::chrono::milliseconds(500)
                );
                response->product_code = dict.from_raw<uint32_t>(product_data);
            } catch (const std::exception& e) {
                RCLCPP_DEBUG(this->get_logger(), "Failed to read product code: %s", e.what());
            }

            // Read revision number (0x1018.03)
            try {
                auto revision_data = sdo_client->read_object(
                    "identity_revision_number",
                    std::chrono::milliseconds(500)
                );
                response->revision_number = dict.from_raw<uint32_t>(revision_data);
            } catch (const std::exception& e) {
                RCLCPP_DEBUG(this->get_logger(), "Failed to read revision number: %s", e.what());
            }

            // Supported operation modes - commonly supported by most motors
            // This is a simplified list; ideally should be read from object dictionary
            response->supported_operation_modes = {1, 3, 8, 9};  // PP, PV, CSP, CSV

            response->success = true;
            response->message = "Motor info retrieved successfully";

            RCLCPP_INFO(this->get_logger(),
                "Motor info for %s (node %d): state=%s, statusword=0x%04X, device_type=0x%08X",
                response->motor_name.c_str(), node_id, response->state.c_str(),
                response->statusword, response->device_type);

        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Get motor info failed: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

// =============================================================================
// EnableMotor Action Handlers
// =============================================================================

    rclcpp_action::GoalResponse MotorDriverNode::handle_enable_goal(
        uint8_t node_id,
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const EnableMotor::Goal> goal) {
        // TODO: Implement enable motor goal handler
        RCLCPP_INFO(this->get_logger(),
            "Enable motor goal received for node_id=%d - TODO", node_id);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse MotorDriverNode::handle_enable_cancel(
        uint8_t node_id,
        std::shared_ptr<EnableMotorGoalHandle> goal_handle) {
        // TODO: Implement enable motor cancel handler
        RCLCPP_INFO(this->get_logger(),
            "Enable motor cancel requested for node_id=%d - TODO", node_id);
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void MotorDriverNode::handle_enable_accepted(
        uint8_t node_id,
        std::shared_ptr<EnableMotorGoalHandle> goal_handle) {
        RCLCPP_INFO(this->get_logger(),
            "Enable motor accepted for node_id=%d, starting execution thread", node_id);

        // Spawn execution thread (non-blocking)
        std::thread{std::bind(&MotorDriverNode::execute_enable, this, node_id,
            goal_handle)}.detach();
    }

    void MotorDriverNode::execute_enable(
        uint8_t node_id,
        std::shared_ptr<EnableMotorGoalHandle> goal_handle) {

        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<EnableMotor::Result>();
        auto feedback = std::make_shared<EnableMotor::Feedback>();

        const auto start_time = this->now();
        const auto timeout_duration = rclcpp::Duration::from_seconds(
            goal->timeout_sec > 0 ? goal->timeout_sec : 10.0
        );

        RCLCPP_INFO(this->get_logger(),
            "Executing enable motor for node_id=%d with timeout=%.1fs",
            node_id, timeout_duration.seconds());

        // Validate motor instance
        auto motor = get_motor(node_id);
        if (!motor) {
            result->success = false;
            result->message = "Invalid node_id: " + std::to_string(node_id);
            result->final_state = "UNKNOWN";
            result->elapsed_time_sec = (this->now() - start_time).seconds();
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
            return;
        }

        auto fsm = motor->get_fsm();

        // Initial state check
        feedback->progress_percent = 0.0;
        feedback->transition_name = "Initial state check";
        update_fsm_feedback(motor, feedback, true);
        goal_handle->publish_feedback(feedback);

        RCLCPP_INFO(this->get_logger(),
            "Motor %d initial state: %s (statusword=0x%04X)",
            node_id, feedback->current_state.c_str(), feedback->statusword);

        // Transition 1: Shutdown -> READY_TO_SWITCH_ON
        if (should_abort_action<EnableMotor>(goal_handle, start_time, timeout_duration,
            "Enable motor")) {
            return;
        }

        if (!execute_state_transition(node_id, motor, feedback, goal_handle,
            "Shutdown (transition to READY_TO_SWITCH_ON)", 20.0,
            [](auto fsm) {
                return fsm->shutdown();
            })) {
            result->success = false;
            result->message = "Failed at step: Shutdown";
            result->final_state = feedback->current_state;
            result->final_statusword = feedback->statusword;
            result->elapsed_time_sec = (this->now() - start_time).seconds();
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(),
                "Motor %d: %s (state=%s)", node_id, result->message.c_str(),
                result->final_state.c_str());
            return;
        }

        // Transition 2: Switch On -> SWITCHED_ON
        if (should_abort_action<EnableMotor>(goal_handle, start_time, timeout_duration,
            "Enable motor")) {
            return;
        }

        if (!execute_state_transition(node_id, motor, feedback, goal_handle,
            "Switch On (transition to SWITCHED_ON)", 50.0,
            [](auto fsm) {
                return fsm->switch_on();
            })) {
            result->success = false;
            result->message = "Failed at step: Switch On";
            result->final_state = feedback->current_state;
            result->final_statusword = feedback->statusword;
            result->elapsed_time_sec = (this->now() - start_time).seconds();
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(),
                "Motor %d: %s (state=%s)", node_id, result->message.c_str(),
                result->final_state.c_str());
            return;
        }

        // Transition 3: Enable Operation -> OPERATION_ENABLED
        if (should_abort_action<EnableMotor>(goal_handle, start_time, timeout_duration,
            "Enable motor")) {
            return;
        }

        if (!execute_state_transition(node_id, motor, feedback, goal_handle,
            "Enable Operation (transition to OPERATION_ENABLED)", 80.0,
            [](auto fsm) {
                return fsm->enable_operation();
            })) {
            result->success = false;
            result->message = "Failed at step: Enable Operation";
            result->final_state = feedback->current_state;
            result->final_statusword = feedback->statusword;
            result->elapsed_time_sec = (this->now() - start_time).seconds();
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(),
                "Motor %d: %s (state=%s)", node_id, result->message.c_str(),
                result->final_state.c_str());
            return;
        }

        // Verify final state and complete
        auto final_state = fsm->get_current_state(true);
        result->final_state = canopen::cia402::get_state_description(final_state);
        result->final_statusword = fsm->get_statusword();
        result->elapsed_time_sec = (this->now() - start_time).seconds();

        if (final_state == canopen::cia402::State::OPERATION_ENABLED) {
            result->success = true;
            result->message = "Motor enabled successfully";

            feedback->current_state = result->final_state;
            feedback->statusword = result->final_statusword;
            feedback->progress_percent = 100.0;
            feedback->transition_name = "Complete";
            goal_handle->publish_feedback(feedback);

            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(),
                "Motor %d enabled successfully in %.2fs (state=%s)",
                node_id, result->elapsed_time_sec, result->final_state.c_str());
        } else {
            result->success = false;
            result->message = "Unexpected final state: " + result->final_state +
                " (expected OPERATION_ENABLED)";
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
        }
    }

// =============================================================================
// ResetFault Action Handlers
// =============================================================================

    rclcpp_action::GoalResponse MotorDriverNode::handle_reset_fault_goal(
        uint8_t node_id,
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const ResetFault::Goal> goal) {
        // TODO: Implement reset fault goal handler
        RCLCPP_INFO(this->get_logger(),
            "Reset fault goal received for node_id=%d - TODO", node_id);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse MotorDriverNode::handle_reset_fault_cancel(
        uint8_t node_id,
        std::shared_ptr<ResetFaultGoalHandle> goal_handle) {
        // TODO: Implement reset fault cancel handler
        RCLCPP_INFO(this->get_logger(),
            "Reset fault cancel requested for node_id=%d - TODO", node_id);
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void MotorDriverNode::handle_reset_fault_accepted(
        uint8_t node_id,
        std::shared_ptr<ResetFaultGoalHandle> goal_handle) {
        RCLCPP_INFO(this->get_logger(),
            "Reset fault accepted for node_id=%d, starting execution thread", node_id);

        // Spawn execution thread (non-blocking)
        std::thread{std::bind(&MotorDriverNode::execute_reset_fault, this, node_id,
            goal_handle)}.detach();
    }

    void MotorDriverNode::execute_reset_fault(
        uint8_t node_id,
        std::shared_ptr<ResetFaultGoalHandle> goal_handle) {

        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<ResetFault::Result>();
        auto feedback = std::make_shared<ResetFault::Feedback>();

        const auto start_time = this->now();
        const auto timeout_duration = rclcpp::Duration::from_seconds(
            goal->timeout_sec > 0 ? goal->timeout_sec : 5.0
        );

        RCLCPP_INFO(this->get_logger(),
            "Executing reset fault for node_id=%d with timeout=%.1fs",
            node_id, timeout_duration.seconds());

        // Validate motor instance
        auto motor = get_motor(node_id);
        if (!motor) {
            result->success = false;
            result->message = "Invalid node_id: " + std::to_string(node_id);
            result->final_state = "UNKNOWN";
            result->elapsed_time_sec = (this->now() - start_time).seconds();
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
            return;
        }

        auto fsm = motor->get_fsm();
        auto sdo_client = motor->get_sdo_client();
        auto& dict = motor->get_dictionary();

        // Read initial error register (object 0x1001)
        try {
            auto error_data = sdo_client->read_object("error_register");
            result->error_register_before = dict.from_raw<uint8_t>(error_data);
            feedback->error_register = result->error_register_before;
            feedback->fault_cleared = false;

            RCLCPP_INFO(this->get_logger(),
                "Motor %d error register before reset: 0x%02X",
                node_id, result->error_register_before);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(),
                "Motor %d: Could not read error register: %s", node_id, e.what());
            result->error_register_before = 0xFF;  // Unknown
        }

        // Get initial state
        update_fsm_feedback(motor, feedback, true);
        goal_handle->publish_feedback(feedback);

        RCLCPP_INFO(this->get_logger(),
            "Motor %d initial state: %s (statusword=0x%04X)",
            node_id, feedback->current_state.c_str(), feedback->statusword);

        // Check if motor is actually in fault state
        if (!fsm->has_fault()) {
            result->success = true;
            result->message = "Motor is not in fault state (no reset needed)";
            result->error_register_after = result->error_register_before;
            result->final_state = feedback->current_state;
            result->elapsed_time_sec = (this->now() - start_time).seconds();

            feedback->fault_cleared = true;
            goal_handle->publish_feedback(feedback);
            goal_handle->succeed(result);

            RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
            return;
        }

        // Check for cancellation/timeout before executing reset
        if (should_abort_action<ResetFault>(goal_handle, start_time, timeout_duration,
            "Reset fault")) {
            return;
        }

        // Execute fault reset
        RCLCPP_INFO(this->get_logger(), "Motor %d: Executing fault reset...", node_id);
        bool reset_success = fsm->reset_fault();

        // Wait briefly for fault to clear
        rclcpp::sleep_for(std::chrono::milliseconds(100));

        // Verify fault cleared
        auto final_state = fsm->get_current_state(true);
        bool fault_cleared = !fsm->has_fault();

        feedback->current_state = canopen::cia402::get_state_description(final_state);
        feedback->statusword = fsm->get_statusword();
        feedback->fault_cleared = fault_cleared;

        // Read final error register
        try {
            auto error_data = sdo_client->read_object("error_register");
            result->error_register_after = dict.from_raw<uint8_t>(error_data);
            feedback->error_register = result->error_register_after;

            RCLCPP_INFO(this->get_logger(),
                "Motor %d error register after reset: 0x%02X",
                node_id, result->error_register_after);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(),
                "Motor %d: Could not read error register after reset: %s", node_id, e.what());
            result->error_register_after = 0xFF;  // Unknown
        }

        result->final_state = feedback->current_state;
        result->elapsed_time_sec = (this->now() - start_time).seconds();

        goal_handle->publish_feedback(feedback);

        if (reset_success && fault_cleared) {
            result->success = true;
            result->message = "Fault reset successfully (state: " + result->final_state + ")";
            goal_handle->succeed(result);

            RCLCPP_INFO(this->get_logger(),
                "Motor %d fault reset successful in %.2fs (error: 0x%02X → 0x%02X, state=%s)",
                node_id, result->elapsed_time_sec,
                result->error_register_before, result->error_register_after,
                result->final_state.c_str());
        } else {
            result->success = false;
            result->message = "Fault reset failed - fault bit still set (state: " +
                result->final_state + ")";
            goal_handle->abort(result);

            RCLCPP_ERROR(this->get_logger(),
                "Motor %d: %s (statusword=0x%04X)",
                node_id, result->message.c_str(), feedback->statusword);
        }
    }

// =============================================================================
// MoveToPosition Action Handlers
// =============================================================================

    rclcpp_action::GoalResponse MotorDriverNode::handle_move_goal(
        uint8_t node_id,
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const MoveToPosition::Goal> goal) {
        // TODO: Implement move to position goal handler
        RCLCPP_INFO(this->get_logger(),
            "Move to position goal received for node_id=%d - TODO", node_id);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse MotorDriverNode::handle_move_cancel(
        uint8_t node_id,
        std::shared_ptr<MoveToPositionGoalHandle> goal_handle) {
        // TODO: Implement move to position cancel handler
        RCLCPP_INFO(this->get_logger(),
            "Move to position cancel requested for node_id=%d - TODO", node_id);
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void MotorDriverNode::handle_move_accepted(
        uint8_t node_id,
        std::shared_ptr<MoveToPositionGoalHandle> goal_handle) {
        RCLCPP_INFO(this->get_logger(),
            "Move to position accepted for node_id=%d, starting execution thread", node_id);

        // Spawn execution thread (non-blocking)
        std::thread{std::bind(&MotorDriverNode::execute_move, this, node_id, goal_handle)}.detach();
    }

    void MotorDriverNode::execute_move(
        uint8_t node_id,
        std::shared_ptr<MoveToPositionGoalHandle> goal_handle) {

        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<MoveToPosition::Result>();
        auto feedback = std::make_shared<MoveToPosition::Feedback>();

        const auto start_time = this->now();
        const auto timeout_duration = rclcpp::Duration::from_seconds(
            goal->timeout_sec > 0 ? goal->timeout_sec : 30.0
        );
        const double position_tolerance = goal->position_tolerance_rad > 0 ?
            goal->position_tolerance_rad : 0.01;  // Default 0.01 rad

        RCLCPP_INFO(this->get_logger(),
            "Executing move to position for node_id=%d: target=%.3f rad, tolerance=%.4f rad, timeout=%.1fs",
            node_id, goal->target_position_rad, position_tolerance, timeout_duration.seconds());

        // Get motor instance
        auto motor = get_motor(node_id);
        if (!motor) {
            result->success = false;
            result->message = "Invalid node_id: " + std::to_string(node_id);
            result->elapsed_time_sec = (this->now() - start_time).seconds();
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
            return;
        }

        auto fsm = motor->get_fsm();
        auto sdo_client = motor->get_sdo_client();
        auto& dict = motor->get_dictionary();

        // 1. Verify motor is enabled
        auto current_state = fsm->get_current_state(true);
        if (current_state != canopen::cia402::State::OPERATION_ENABLED) {
            result->success = false;
            result->message = "Motor not enabled. Current state: " +
                std::string(canopen::cia402::get_state_description(current_state));
            result->elapsed_time_sec = (this->now() - start_time).seconds();
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
            return;
        }

        // 2. Set operation mode to Profile Position (PP = 1)
        RCLCPP_INFO(this->get_logger(), "Motor %d: Setting Profile Position mode...", node_id);
        std::string mode_error_msg;
        if (!set_operation_mode_verified(node_id, motor, 1, mode_error_msg)) {
            result->success = false;
            result->message = mode_error_msg;
            result->elapsed_time_sec = (this->now() - start_time).seconds();
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
            return;
        }

        // 3. Set profile velocity and acceleration if provided
        if (goal->max_velocity_rad_s > 0) {
            try {
                // Convert rad/s to encoder counts/s (using encoder resolution from params)
                // For now, assuming direct value - should use conversion factor
                uint32_t velocity_counts = static_cast<uint32_t>(goal->max_velocity_rad_s * 1000);  // Placeholder
                auto vel_data = dict.to_raw(velocity_counts);
                sdo_client->write_object("profile_velocity", vel_data);
                RCLCPP_DEBUG(this->get_logger(), "Motor %d: Set profile velocity: %u", node_id,
                    velocity_counts);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Motor %d: Could not set profile velocity: %s",
                    node_id, e.what());
            }
        }

        if (goal->max_acceleration_rad_s2 > 0) {
            try {
                uint32_t accel_counts = static_cast<uint32_t>(goal->max_acceleration_rad_s2 * 1000);  // Placeholder
                auto accel_data = dict.to_raw(accel_counts);
                sdo_client->write_object("profile_acceleration", accel_data);
                RCLCPP_DEBUG(this->get_logger(), "Motor %d: Set profile acceleration: %u", node_id,
                    accel_counts);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Motor %d: Could not set profile acceleration: %s",
                    node_id, e.what());
            }
        }

        // 4. Get initial position
        double initial_position_rad = motor->get_position();  // From MotorInstance state
        double total_distance = std::abs(goal->target_position_rad - initial_position_rad);

        RCLCPP_INFO(this->get_logger(),
            "Motor %d: Initial position=%.3f rad, target=%.3f rad, distance=%.3f rad",
            node_id, initial_position_rad, goal->target_position_rad, total_distance);

        // 5. Write target position
        try {
            // Convert radians to encoder counts (assuming encoder_resolution from params)
            // For now using placeholder conversion - should use actual encoder resolution
            int32_t target_counts = static_cast<int32_t>(goal->target_position_rad * 10000);  // Placeholder
            auto target_data = dict.to_raw(target_counts);
            sdo_client->write_object("target_position", target_data);

            RCLCPP_INFO(this->get_logger(),
                "Motor %d: Target position written: %d counts (%.3f rad)",
                node_id, target_counts, goal->target_position_rad);
        } catch (const std::exception& e) {
            result->success = false;
            result->message = "Failed to write target position: " + std::string(e.what());
            result->elapsed_time_sec = (this->now() - start_time).seconds();
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
            return;
        }

        // 6. Start motion by setting bit 4 in controlword (new setpoint)
        try {
            uint16_t controlword = 0x001F;  // Enable operation + new setpoint
            auto cw_data = dict.to_raw(controlword);
            sdo_client->write_object("controlword", cw_data);
            RCLCPP_INFO(this->get_logger(), "Motor %d: Motion started (controlword=0x%04X)",
                node_id, controlword);
        } catch (const std::exception& e) {
            result->success = false;
            result->message = "Failed to start motion: " + std::string(e.what());
            result->elapsed_time_sec = (this->now() - start_time).seconds();
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
            return;
        }

        // 7. Monitor progress via PDO feedback
        rclcpp::Rate feedback_rate(20);  // 20 Hz feedback updates
        bool target_reached = false;

        while (rclcpp::ok() && !target_reached) {
            // Check for cancellation or timeout (with quick stop on abort)
            if (goal_handle->is_canceling() || (this->now() - start_time) > timeout_duration) {
                // Send quick stop
                try {
                    fsm->quick_stop();
                } catch (...) {
                }

                result->final_position_rad = motor->get_position();
                result->position_error_rad = goal->target_position_rad - result->final_position_rad;
                result->target_reached = false;
                result->elapsed_time_sec = (this->now() - start_time).seconds();

                if (goal_handle->is_canceling()) {
                    result->success = false;
                    result->message = "Move to position cancelled by client";
                    result->timed_out = false;
                    goal_handle->canceled(result);
                    RCLCPP_WARN(this->get_logger(), "%s", result->message.c_str());
                } else {
                    result->success = false;
                    result->message = "Move to position timeout after " +
                        std::to_string(result->elapsed_time_sec) + "s";
                    result->timed_out = true;
                    goal_handle->abort(result);
                    RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
                }
                return;
            }

            // Get current state from MotorInstance (updated by PDO callbacks)
            double current_position_rad = motor->get_position();
            double current_velocity_rad_s = motor->get_velocity();
            double distance_to_target = goal->target_position_rad - current_position_rad;
            double distance_traveled = std::abs(current_position_rad - initial_position_rad);

            // Calculate progress
            double progress = (total_distance > 0) ?
                (distance_traveled / total_distance) * 100.0 : 100.0;
            progress = std::min(100.0, std::max(0.0, progress));

            // Update feedback
            feedback->current_position_rad = current_position_rad;
            feedback->current_velocity_rad_s = current_velocity_rad_s;
            feedback->distance_to_target_rad = distance_to_target;
            feedback->progress_percent = static_cast<float>(progress);
            feedback->elapsed_time_sec = (this->now() - start_time).seconds();

            // Check if target reached
            if (std::abs(distance_to_target) <= position_tolerance) {
                feedback->target_reached = true;
                target_reached = true;

                RCLCPP_INFO(this->get_logger(),
                    "Motor %d: Target reached! position=%.3f rad, error=%.4f rad",
                    node_id, current_position_rad, distance_to_target);
            } else {
                feedback->target_reached = false;
            }

            goal_handle->publish_feedback(feedback);

            RCLCPP_DEBUG(this->get_logger(),
                "Motor %d progress: %.1f%%, position=%.3f rad, distance=%.3f rad",
                node_id, progress, current_position_rad, distance_to_target);

            feedback_rate.sleep();
        }

        // 8. Success
        result->success = true;
        result->message = "Target position reached successfully";
        result->final_position_rad = motor->get_position();
        result->position_error_rad = goal->target_position_rad - result->final_position_rad;
        result->target_reached = true;
        result->timed_out = false;
        result->elapsed_time_sec = (this->now() - start_time).seconds();

        goal_handle->succeed(result);

        RCLCPP_INFO(this->get_logger(),
            "Motor %d move completed in %.2fs (final position=%.3f rad, error=%.4f rad)",
            node_id, result->elapsed_time_sec, result->final_position_rad,
            result->position_error_rad);
    }

}  // namespace ros2_waveshare
