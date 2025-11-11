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

#include "ros2_waveshare/motor_driver_node.hpp"
#include <canopen/cia402_constants.hpp>
#include <canopen/pdo_constants.hpp>

using namespace std::chrono_literals;

namespace ros2_waveshare {

// =============================================================================
// PDO Reception Callbacks
// =============================================================================

/**
 * @brief TPDO1 receive callback - Statusword + Position feedback
 *
 * Typical CIA402 TPDO1 mapping:
 * - Bytes 0-1: Statusword (0x6041)
 * - Bytes 2-5: Position Actual Value (0x6064) - int32_t
 */
    void MotorDriverNode::on_tpdo1_received(uint8_t node_id, const can_frame& frame) {
        auto motor = get_motor(node_id);
        if (!motor) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Received TPDO1 for unknown motor %d", node_id);
            return;
        }

        // Parse TPDO1 data using library helpers
        if (frame.can_dlc < 6) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "TPDO1 from motor %d has insufficient data (expected 6 bytes, got %d)",
                node_id, frame.can_dlc);
            return;
        }

        auto& dict = motor->get_dictionary();

        // Extract statusword (bytes 0-1)
        std::vector<uint8_t> statusword_data(frame.data, frame.data + 2);
        uint16_t statusword = dict.from_raw<uint16_t>(statusword_data);

        // Extract position (bytes 2-5)
        std::vector<uint8_t> position_data(frame.data + 2, frame.data + 6);
        int32_t position_counts = dict.from_raw<int32_t>(position_data);

        // Update motor state
        motor->update_statusword(statusword);
        motor->update_position(position_counts);
        motor->update_last_tpdo1_time(this->now());

        // Decode state for logging if enabled
        if (log_state_transitions_) {
            auto state = canopen::cia402::decode_statusword(statusword);
            auto prev_state = motor->get_previous_state();

            if (state != prev_state) {
                RCLCPP_INFO(this->get_logger(),
                    "Motor %d (%s) state changed: %s → %s (statusword: 0x%04X)",
                    node_id, motor->get_name().c_str(),
                    canopen::cia402::get_state_description(prev_state),
                    canopen::cia402::get_state_description(state),
                    statusword);
                motor->set_previous_state(state);
            }
        }

        // Publish motor feedback (individual motor topic)
        if (auto pub = motor->get_feedback_publisher()) {
            auto msg = build_motor_feedback(node_id);
            pub->publish(msg);
        }
    }

/**
 * @brief TPDO2 receive callback - Velocity + Current feedback
 *
 * Typical CIA402 TPDO2 mapping:
 * - Bytes 0-3: Velocity Actual Value (0x606C) - int32_t
 * - Bytes 4-5: Current Actual Value (0x6078) - int16_t
 */
    void MotorDriverNode::on_tpdo2_received(uint8_t node_id, const can_frame& frame) {
        auto motor = get_motor(node_id);
        if (!motor) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Received TPDO2 for unknown motor %d", node_id);
            return;
        }

        // Parse TPDO2 data using library helpers
        if (frame.can_dlc < 6) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "TPDO2 from motor %d has insufficient data (expected 6 bytes, got %d)",
                node_id, frame.can_dlc);
            return;
        }

        auto& dict = motor->get_dictionary();

        // Extract velocity (bytes 0-3)
        std::vector<uint8_t> velocity_data(frame.data, frame.data + 4);
        int32_t velocity_counts = dict.from_raw<int32_t>(velocity_data);

        // Extract current (bytes 4-5)
        std::vector<uint8_t> current_data(frame.data + 4, frame.data + 6);
        int16_t current_ma = dict.from_raw<int16_t>(current_data);

        // Update motor state
        motor->update_velocity(velocity_counts);
        motor->update_current(current_ma);
        motor->update_last_tpdo2_time(this->now());
    }

// =============================================================================
// Command Subscriber Callback
// =============================================================================

/**
 * @brief Process motor command - convert to RPDO and send to motor
 *
 * This callback receives MotorCommand messages and converts them to CANopen
 * RPDOs for transmission to the motor controller.
 */
    void MotorDriverNode::on_motor_command(uint8_t node_id, const MotorCommand::SharedPtr msg) {
        auto motor = get_motor(node_id);
        if (!motor) {
            RCLCPP_WARN(this->get_logger(),
                "Received command for unknown motor %d", node_id);
            return;
        }

        // Check if motor is operational
        if (!motor->is_enabled()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Motor %d (%s) not enabled, ignoring command",
                node_id, motor->get_name().c_str());
            return;
        }

        auto& dict = motor->get_dictionary();
        std::vector<uint8_t> rpdo_data;

        // Build RPDO1 based on operation mode
        // RPDO1: Controlword (2 bytes) + Target Position/Velocity (4 bytes)

        // Add controlword (using library helper)
        uint16_t controlword = msg->controlword;
        auto controlword_bytes = dict.to_raw(controlword);
        rpdo_data.insert(rpdo_data.end(), controlword_bytes.begin(), controlword_bytes.end());

        // Add target position or velocity based on mode
        int8_t mode = motor->get_operation_mode();

        if (mode == 1 || mode == -1) {
            // Profile Position (PP) or Interpolated Position (IP)
            int32_t target_counts = rad_to_counts(node_id, msg->target_position_rad);
            auto position_bytes = dict.to_raw(target_counts);
            rpdo_data.insert(rpdo_data.end(), position_bytes.begin(), position_bytes.end());

        } else if (mode == 3 || mode == -3) {
            // Profile Velocity (PV) or Cyclic Sync Velocity (CSV)
            int32_t target_velocity = rad_per_sec_to_counts_per_sec(node_id,
                msg->target_velocity_rad_s);
            auto velocity_bytes = dict.to_raw(target_velocity);
            rpdo_data.insert(rpdo_data.end(), velocity_bytes.begin(), velocity_bytes.end());

        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Motor %d: Unsupported operation mode %d for command",
                node_id, mode);
            return;
        }

        // Send RPDO1
        if (!pdo_manager_->send_rpdo1(node_id, rpdo_data)) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Failed to send RPDO1 to motor %d", node_id);
        }
    }

// =============================================================================
// Timer Callbacks
// =============================================================================

/**
 * @brief Send SYNC message to synchronize PDO transmission
 *
 * The SYNC message triggers TPDOs on all motors configured for synchronous
 * operation. This ensures coordinated feedback updates.
 */
    void MotorDriverNode::send_sync_timer_callback() {
        if (!pdo_manager_) {
            return;
        }

        // Send SYNC message (COB-ID 0x080)
        can_frame sync_frame;
        sync_frame.can_id = canopen::pdo::to_cob_base(canopen::pdo::PDOCobIDBase::SYNC);
        sync_frame.can_dlc = 0; // SYNC has no data payload

        // Use the PDO manager's socket to send SYNC
        // Note: PDOManager doesn't have a send_sync method, so we'll use the socket directly
        if (can_socket_ && can_socket_->is_open()) {
            ssize_t bytes_sent = can_socket_->send(sync_frame);
            if (bytes_sent != sizeof(can_frame)) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Failed to send SYNC message");
            }
        }
    }

/**
 * @brief Publish combined JointState message for all motors
 *
 * This publishes to /joint_states topic which is consumed by robot_state_publisher
 * for TF transforms and URDF visualization.
 */
    void MotorDriverNode::publish_joint_state_callback() {
        if (!pub_joint_states_) {
            return;
        }

        auto msg = build_joint_state_msg();
        pub_joint_states_->publish(msg);
    }

/**
 * @brief Publish diagnostics and PDO statistics
 *
 * Publishes diagnostic information for all motors and PDO communication statistics.
 */
    void MotorDriverNode::publish_diagnostics_callback() {
        // Publish diagnostics
        if (pub_diagnostics_) {
            auto diag_msg = build_diagnostics_msg();
            pub_diagnostics_->publish(diag_msg);
        }

        // Publish PDO statistics
        if (pub_pdo_stats_ && log_pdo_statistics_) {
            auto stats_msg = build_pdo_statistics();
            pub_pdo_stats_->publish(stats_msg);
        }
    }

// =============================================================================
// Helper Methods - Unit Conversions
// =============================================================================

    double MotorDriverNode::counts_to_rad(uint8_t node_id, int32_t counts) const {
        auto motor = get_motor(node_id);
        if (!motor) return 0.0;

        double counts_per_rev = motor->get_counts_per_revolution();
        double gear_ratio = motor->get_gear_ratio();

        // radians = counts * (2π / counts_per_rev) / gear_ratio
        return static_cast<double>(counts) * (2.0 * M_PI) / (counts_per_rev * gear_ratio);
    }

    int32_t MotorDriverNode::rad_to_counts(uint8_t node_id, double rad) const {
        auto motor = get_motor(node_id);
        if (!motor) return 0;

        double counts_per_rev = motor->get_counts_per_revolution();
        double gear_ratio = motor->get_gear_ratio();

        // counts = radians * (counts_per_rev * gear_ratio) / (2π)
        return static_cast<int32_t>(rad * (counts_per_rev * gear_ratio) / (2.0 * M_PI));
    }

    double MotorDriverNode::counts_per_sec_to_rad_per_sec(uint8_t node_id,
        int32_t counts_per_sec) const {
        auto motor = get_motor(node_id);
        if (!motor) return 0.0;

        double counts_per_rev = motor->get_counts_per_revolution();
        double gear_ratio = motor->get_gear_ratio();

        // rad/s = counts/s * (2π / counts_per_rev) / gear_ratio
        return static_cast<double>(counts_per_sec) * (2.0 * M_PI) / (counts_per_rev * gear_ratio);
    }

    int32_t MotorDriverNode::rad_per_sec_to_counts_per_sec(uint8_t node_id,
        double rad_per_sec) const {
        auto motor = get_motor(node_id);
        if (!motor) return 0;

        double counts_per_rev = motor->get_counts_per_revolution();
        double gear_ratio = motor->get_gear_ratio();

        // counts/s = rad/s * (counts_per_rev * gear_ratio) / (2π)
        return static_cast<int32_t>(rad_per_sec * (counts_per_rev * gear_ratio) / (2.0 * M_PI));
    }

    double MotorDriverNode::current_to_torque(uint8_t node_id, int16_t current_ma) const {
        auto motor = get_motor(node_id);
        if (!motor) return 0.0;

        double torque_constant = motor->get_torque_constant();
        double gear_ratio = motor->get_gear_ratio();

        // Torque = (current_A * torque_constant) * gear_ratio
        double current_a = static_cast<double>(current_ma) / 1000.0;
        return current_a * torque_constant * gear_ratio;
    }

    int16_t MotorDriverNode::torque_to_current(uint8_t node_id, double torque_nm) const {
        auto motor = get_motor(node_id);
        if (!motor) return 0;

        double torque_constant = motor->get_torque_constant();
        double gear_ratio = motor->get_gear_ratio();

        // Current = (torque / gear_ratio) / torque_constant
        double current_a = (torque_nm / gear_ratio) / torque_constant;
        return static_cast<int16_t>(current_a * 1000.0);
    }

// =============================================================================
// Helper Methods - Message Building
// =============================================================================

    MotorDriverNode::MotorFeedback MotorDriverNode::build_motor_feedback(uint8_t node_id) {
        MotorFeedback msg;
        auto motor = get_motor(node_id);

        if (!motor) {
            RCLCPP_ERROR(this->get_logger(), "Cannot build feedback for unknown motor %d", node_id);
            return msg;
        }

        msg.header.stamp = this->now();
        msg.header.frame_id = motor->get_name();

        msg.motor_name = motor->get_name();
        msg.node_id = node_id;

        // Position and velocity (converted to radians and rad/s)
        msg.position_rad = counts_to_rad(node_id, motor->get_position());
        msg.velocity_rad_s = counts_per_sec_to_rad_per_sec(node_id, motor->get_velocity());

        // Torque (converted from current)
        msg.torque_nm = current_to_torque(node_id, motor->get_current());

        // Raw values
        msg.encoder_counts = motor->get_position();
        msg.velocity_counts_per_sec = motor->get_velocity();
        msg.current_ma = motor->get_current();

        // State information
        msg.statusword = motor->get_statusword();
        auto state = canopen::cia402::decode_statusword(msg.statusword);
        msg.state = canopen::cia402::get_state_description(state);
        msg.operation_mode_display = motor->get_operation_mode();

        return msg;
    }

    MotorDriverNode::MotorStatus MotorDriverNode::build_motor_status(uint8_t node_id) {
        MotorStatus msg;
        auto motor = get_motor(node_id);

        if (!motor) {
            return msg;
        }

        msg.header.stamp = this->now();
        msg.motor_name = motor->get_name();
        msg.node_id = node_id;

        auto state = canopen::cia402::decode_statusword(motor->get_statusword());
        msg.state = canopen::cia402::get_state_description(state);
        msg.fault = (state == canopen::cia402::State::FAULT);
        msg.operational = (state == canopen::cia402::State::OPERATION_ENABLED);

        return msg;
    }

    MotorDriverNode::JointState MotorDriverNode::build_joint_state_msg() {
        JointState msg;
        msg.header.stamp = this->now();

        for (const auto& [node_id, motor] : motors_) {
            msg.name.push_back(motor->get_name());
            msg.position.push_back(counts_to_rad(node_id, motor->get_position()));
            msg.velocity.push_back(counts_per_sec_to_rad_per_sec(node_id, motor->get_velocity()));
            msg.effort.push_back(current_to_torque(node_id, motor->get_current()));
        }

        return msg;
    }

    MotorDriverNode::PDOStatistics MotorDriverNode::build_pdo_statistics() {
        PDOStatistics msg;
        msg.header.stamp = this->now();

        if (!pdo_manager_) {
            return msg;
        }

        // PDOStatistics message has single values, not arrays
        // We'll aggregate statistics from all motors
        uint64_t total_tpdo1 = 0;
        uint64_t total_tpdo2 = 0;
        uint64_t total_rpdo1 = 0;
        uint64_t total_rpdo2 = 0;
        uint64_t total_errors = 0;
        double sum_latency = 0.0;
        size_t latency_count = 0;

        for (const auto& [node_id, motor] : motors_) {
            auto stats = pdo_manager_->get_statistics(node_id);
            total_tpdo1 += stats.tpdo1_received;
            total_tpdo2 += stats.tpdo2_received;
            total_rpdo1 += stats.rpdo1_sent;
            total_rpdo2 += stats.rpdo2_sent;
            total_errors += stats.errors;
            sum_latency += stats.avg_latency_us;
            latency_count++;
        }

        msg.tpdo1_received = total_tpdo1;
        msg.tpdo2_received = total_tpdo2;
        msg.rpdo1_sent = total_rpdo1;
        msg.rpdo2_sent = total_rpdo2;
        msg.errors = total_errors;
        msg.avg_latency_us = (latency_count > 0) ? (sum_latency / latency_count) : 0.0;

        return msg;
    }

    MotorDriverNode::DiagnosticArray MotorDriverNode::build_diagnostics_msg() {
        DiagnosticArray msg;
        msg.header.stamp = this->now();

        for (const auto& [node_id, motor] : motors_) {
            DiagnosticStatus status;
            status.name = "Motor " + motor->get_name();
            status.hardware_id = "CAN_" + std::to_string(node_id);

            // Determine diagnostic level
            auto state = canopen::cia402::decode_statusword(motor->get_statusword());

            if (state == canopen::cia402::State::FAULT) {
                status.level = DiagnosticStatus::ERROR;
                status.message = "Motor in FAULT state";
            } else if (!motor->is_enabled()) {
                status.level = DiagnosticStatus::WARN;
                status.message = "Motor not enabled";
            } else if (state == canopen::cia402::State::OPERATION_ENABLED) {
                status.level = DiagnosticStatus::OK;
                status.message = "Motor operational";
            } else {
                status.level = DiagnosticStatus::WARN;
                status.message = std::string("Motor in state: ") +
                    canopen::cia402::get_state_description(state);
            }

            // Add key-value pairs
            KeyValue kv;

            kv.key = "State";
            kv.value = canopen::cia402::get_state_description(state);
            status.values.push_back(kv);

            kv.key = "Statusword";
            kv.value = "0x" + std::to_string(motor->get_statusword());
            status.values.push_back(kv);

            kv.key = "Position (rad)";
            kv.value = std::to_string(counts_to_rad(node_id, motor->get_position()));
            status.values.push_back(kv);

            kv.key = "Velocity (rad/s)";
            kv.value = std::to_string(counts_per_sec_to_rad_per_sec(node_id,
                motor->get_velocity()));
            status.values.push_back(kv);

            kv.key = "Current (mA)";
            kv.value = std::to_string(motor->get_current());
            status.values.push_back(kv);

            msg.status.push_back(status);
        }

        return msg;
    }

}  // namespace ros2_waveshare
