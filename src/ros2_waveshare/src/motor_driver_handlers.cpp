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
        // TODO: Implement enable motor accepted handler
        RCLCPP_INFO(this->get_logger(),
            "Enable motor accepted for node_id=%d - TODO", node_id);

        // For now, just succeed immediately
        auto result = std::make_shared<EnableMotor::Result>();
        result->success = false;
        result->message = "Enable motor not yet implemented";
        goal_handle->succeed(result);
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
        // TODO: Implement reset fault accepted handler
        RCLCPP_INFO(this->get_logger(),
            "Reset fault accepted for node_id=%d - TODO", node_id);

        // For now, just succeed immediately
        auto result = std::make_shared<ResetFault::Result>();
        result->success = false;
        result->message = "Reset fault not yet implemented";
        goal_handle->succeed(result);
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
        // TODO: Implement move to position accepted handler
        RCLCPP_INFO(this->get_logger(),
            "Move to position accepted for node_id=%d - TODO", node_id);

        // For now, just succeed immediately
        auto result = std::make_shared<MoveToPosition::Result>();
        result->success = false;
        result->message = "Move to position not yet implemented";
        goal_handle->succeed(result);
    }

}  // namespace ros2_waveshare
