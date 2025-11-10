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

#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Standard ROS2 messages
#include <sensor_msgs/msg/joint_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

// Custom ROS2 messages
#include <ros2_waveshare_msgs/msg/motor_feedback.hpp>
#include <ros2_waveshare_msgs/msg/motor_command.hpp>
#include <ros2_waveshare_msgs/msg/motor_status.hpp>
#include <ros2_waveshare_msgs/msg/pdo_statistics.hpp>

// Custom ROS2 services
#include <ros2_waveshare_msgs/srv/sdo_read.hpp>
#include <ros2_waveshare_msgs/srv/sdo_write.hpp>
#include <ros2_waveshare_msgs/srv/set_operation_mode.hpp>
#include <ros2_waveshare_msgs/srv/get_motor_info.hpp>

// Custom ROS2 actions
#include <ros2_waveshare_msgs/action/enable_motor.hpp>
#include <ros2_waveshare_msgs/action/reset_fault.hpp>
#include <ros2_waveshare_msgs/action/move_to_position.hpp>

// Waveshare CANopen library
#include <canopen/sdo_client.hpp>
#include <canopen/pdo_manager.hpp>
#include <canopen/cia402_fsm.hpp>
#include <canopen/object_dictionary.hpp>
#include <io/can_socket.hpp>
#include <io/real_can_socket.hpp>

// Motor instance class
#include <ros2_waveshare/motor_instance.hpp>

#include <linux/can.h>

namespace ros2_waveshare {

/**
 * @brief ROS2 node for controlling multiple CANopen motors via waveshare_cpp library
 *
 * Features:
 * - Manages multiple motors (up to 4) on single CAN bus
 * - Publishes motor feedback (100Hz) and standard JointState messages
 * - Subscribes to motor commands and executes via RPDO
 * - Provides SDO read/write services for register access
 * - Implements action servers for enable, fault reset, position moves
 * - Handles unit conversion (encoder counts → radians, current → torque)
 * - Thread-safe operation with PDO receive thread
 */
    class MotorDriverNode : public rclcpp::Node {
        public:
            using MotorFeedback = ros2_waveshare_msgs::msg::MotorFeedback;
            using MotorCommand = ros2_waveshare_msgs::msg::MotorCommand;
            using MotorStatus = ros2_waveshare_msgs::msg::MotorStatus;
            using PDOStatistics = ros2_waveshare_msgs::msg::PDOStatistics;

            using SDORead = ros2_waveshare_msgs::srv::SDORead;
            using SDOWrite = ros2_waveshare_msgs::srv::SDOWrite;
            using SetOperationMode = ros2_waveshare_msgs::srv::SetOperationMode;
            using GetMotorInfo = ros2_waveshare_msgs::srv::GetMotorInfo;

            using EnableMotor = ros2_waveshare_msgs::action::EnableMotor;
            using ResetFault = ros2_waveshare_msgs::action::ResetFault;
            using MoveToPosition = ros2_waveshare_msgs::action::MoveToPosition;

            using EnableMotorGoalHandle = rclcpp_action::ServerGoalHandle<EnableMotor>;
            using ResetFaultGoalHandle = rclcpp_action::ServerGoalHandle<ResetFault>;
            using MoveToPositionGoalHandle = rclcpp_action::ServerGoalHandle<MoveToPosition>;

            using JointState = sensor_msgs::msg::JointState;
            using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
            using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
            using KeyValue = diagnostic_msgs::msg::KeyValue;

            /**
             * @brief Constructor
             * @param options Node options (allow setting parameters, etc.)
             */
            explicit MotorDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

            /**
             * @brief Destructor - cleanup CAN resources
             */
            ~MotorDriverNode();

        private:
            // =========================================================================
            // Core Components (Shared across all motors)
            // =========================================================================

            // CAN socket (shared by all motors on same bus)
            std::shared_ptr<waveshare::ICANSocket> can_socket_;

            // PDO Manager (single receive thread for all motors)
            std::unique_ptr<canopen::PDOManager> pdo_manager_;

            // Motor instances (map: node_id → MotorInstance)
            std::map<uint8_t, std::unique_ptr<MotorInstance> > motors_;

            // Combined publishers (all motors)
            rclcpp::Publisher<JointState>::SharedPtr pub_joint_states_;
            rclcpp::Publisher<PDOStatistics>::SharedPtr pub_pdo_stats_;
            rclcpp::Publisher<DiagnosticArray>::SharedPtr pub_diagnostics_;

            // =========================================================================
            // Timers
            // =========================================================================

            rclcpp::TimerBase::SharedPtr timer_sync_;   // SYNC message (100Hz)
            rclcpp::TimerBase::SharedPtr timer_joint_state_; // JointState publishing
            rclcpp::TimerBase::SharedPtr timer_diagnostics_; // Diagnostics publishing

            // =========================================================================
            // Parameters (cached from ROS2 parameter server)
            // =========================================================================

            std::string can_interface_;

            // PDO configuration
            double sync_rate_hz_;
            bool sync_enabled_;
            int tpdo_timeout_ms_;

            // Publishing rates
            double joint_state_rate_hz_;
            double pdo_statistics_rate_hz_;
            double diagnostics_rate_hz_;

            // Timeouts
            int sdo_timeout_ms_;
            int state_transition_timeout_ms_;
            double action_default_timeout_sec_;

            // Safety
            bool enable_watchdog_;
            int watchdog_timeout_ms_;
            bool auto_disable_on_fault_;

            // Diagnostics thresholds
            double temperature_warning_c_;
            double temperature_error_c_;
            double voltage_min_warning_v_;
            double voltage_max_warning_v_;

            // Logging
            bool log_sdo_transactions_;
            bool log_pdo_statistics_;
            bool log_state_transitions_;

            // =========================================================================
            // Initialization Methods
            // =========================================================================

            /**
             * @brief Load all parameters from ROS2 parameter server
             * @throws std::runtime_error if required parameters missing
             */
            void load_parameters();

            /**
             * @brief Open CAN socket and verify connection
             * @throws std::runtime_error if CAN interface unavailable
             */
            void initialize_can_socket();

            /**
             * @brief Initialize all motors (SDO, FSM, publishers, subscribers, services, actions)
             * @throws std::runtime_error if motor initialization fails
             */
            void initialize_motors();

            /**
             * @brief Initialize PDO manager and register TPDO callbacks
             */
            void initialize_pdo_manager();

            /**
             * @brief Setup combined publishers (JointState, statistics, diagnostics)
             */
            void setup_publishers();

            /**
             * @brief Start all timers (SYNC, JointState, diagnostics)
             */
            void setup_timers();

            /**
             * @brief Initialize a single motor instance
             * @param node_id CANopen node ID (1-4)
             * @param motor_params Parameter map for this motor
             */
            void initialize_motor(uint8_t node_id,
                const std::map<std::string, rclcpp::Parameter>& motor_params);

            // =========================================================================
            // PDO Callbacks (per motor)
            // =========================================================================

            /**
             * @brief TPDO1 receive callback (statusword, position)
             * @param node_id Motor node ID
             * @param frame CAN frame containing TPDO1 data
             */
            void on_tpdo1_received(uint8_t node_id, const can_frame& frame);

            /**
             * @brief TPDO2 receive callback (velocity, current)
             * @param node_id Motor node ID
             * @param frame CAN frame containing TPDO2 data
             */
            void on_tpdo2_received(uint8_t node_id, const can_frame& frame);

            // =========================================================================
            // Command Subscriber Callback
            // =========================================================================

            /**
             * @brief Process motor command (convert to RPDO and send to motor)
             * @param node_id Motor node ID
             * @param msg MotorCommand message
             */
            void on_motor_command(uint8_t node_id, const MotorCommand::SharedPtr msg);

            // =========================================================================
            // Service Callbacks
            // =========================================================================

            void handle_sdo_read(
                uint8_t node_id,
                const std::shared_ptr<SDORead::Request> request,
                std::shared_ptr<SDORead::Response> response);

            void handle_sdo_write(
                uint8_t node_id,
                const std::shared_ptr<SDOWrite::Request> request,
                std::shared_ptr<SDOWrite::Response> response);

            void handle_set_operation_mode(
                uint8_t node_id,
                const std::shared_ptr<SetOperationMode::Request> request,
                std::shared_ptr<SetOperationMode::Response> response);

            void handle_get_motor_info(
                uint8_t node_id,
                const std::shared_ptr<GetMotorInfo::Request> request,
                std::shared_ptr<GetMotorInfo::Response> response);

            // =========================================================================
            // Action Server Callbacks - EnableMotor
            // =========================================================================

            rclcpp_action::GoalResponse handle_enable_goal(
                uint8_t node_id,
                const rclcpp_action::GoalUUID& uuid,
                std::shared_ptr<const EnableMotor::Goal> goal);

            rclcpp_action::CancelResponse handle_enable_cancel(
                uint8_t node_id,
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<EnableMotor> > goal_handle);

            void handle_enable_accepted(
                uint8_t node_id,
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<EnableMotor> > goal_handle);

            void execute_enable(
                uint8_t node_id,
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<EnableMotor> > goal_handle);

            // =========================================================================
            // Action Server Callbacks - ResetFault
            // =========================================================================

            rclcpp_action::GoalResponse handle_reset_fault_goal(
                uint8_t node_id,
                const rclcpp_action::GoalUUID& uuid,
                std::shared_ptr<const ResetFault::Goal> goal);

            rclcpp_action::CancelResponse handle_reset_fault_cancel(
                uint8_t node_id,
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetFault> > goal_handle);

            void handle_reset_fault_accepted(
                uint8_t node_id,
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetFault> > goal_handle);

            void execute_reset_fault(
                uint8_t node_id,
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetFault> > goal_handle);

            // =========================================================================
            // Action Server Callbacks - MoveToPosition
            // =========================================================================

            rclcpp_action::GoalResponse handle_move_goal(
                uint8_t node_id,
                const rclcpp_action::GoalUUID& uuid,
                std::shared_ptr<const MoveToPosition::Goal> goal);

            rclcpp_action::CancelResponse handle_move_cancel(
                uint8_t node_id,
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPosition> > goal_handle);

            void handle_move_accepted(
                uint8_t node_id,
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPosition> > goal_handle);

            void execute_move(
                uint8_t node_id,
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPosition> > goal_handle);

            // =========================================================================
            // Timer Callbacks
            // =========================================================================

            /**
             * @brief Send SYNC message to all motors (100Hz)
             */
            void send_sync_timer_callback();

            /**
             * @brief Publish combined JointState for all motors
             */
            void publish_joint_state_callback();

            /**
             * @brief Publish diagnostics and PDO statistics (1Hz)
             */
            void publish_diagnostics_callback();

            // =========================================================================
            // Helper Methods - Unit Conversions
            // =========================================================================

            /**
             * @brief Convert encoder counts to radians
             * @param node_id Motor node ID (for parameter lookup)
             * @param counts Raw encoder counts
             * @return Position in radians
             */
            double counts_to_rad(uint8_t node_id, int32_t counts) const;

            /**
             * @brief Convert radians to encoder counts
             * @param node_id Motor node ID (for parameter lookup)
             * @param rad Position in radians
             * @return Encoder counts
             */
            int32_t rad_to_counts(uint8_t node_id, double rad) const;

            /**
             * @brief Convert encoder counts/sec to rad/sec
             * @param node_id Motor node ID
             * @param counts_per_sec Velocity in counts/sec
             * @return Velocity in rad/sec
             */
            double counts_per_sec_to_rad_per_sec(uint8_t node_id, int32_t counts_per_sec) const;

            /**
             * @brief Convert rad/sec to encoder counts/sec
             * @param node_id Motor node ID
             * @param rad_per_sec Velocity in rad/sec
             * @return Velocity in counts/sec
             */
            int32_t rad_per_sec_to_counts_per_sec(uint8_t node_id, double rad_per_sec) const;

            /**
             * @brief Convert current to torque
             * @param node_id Motor node ID (for torque constant)
             * @param current_ma Current in milliamps
             * @return Torque in N·m
             */
            double current_to_torque(uint8_t node_id, int16_t current_ma) const;

            /**
             * @brief Convert torque to current
             * @param node_id Motor node ID
             * @param torque_nm Torque in N·m
             * @return Current in milliamps
             */
            int16_t torque_to_current(uint8_t node_id, double torque_nm) const;

            // =========================================================================
            // Helper Methods - Message Building
            // =========================================================================

            /**
             * @brief Build MotorFeedback message from TPDO data
             * @param node_id Motor node ID
             * @return Populated MotorFeedback message
             */
            MotorFeedback build_motor_feedback(uint8_t node_id);

            /**
             * @brief Build MotorStatus message (lightweight)
             * @param node_id Motor node ID
             * @return Populated MotorStatus message
             */
            MotorStatus build_motor_status(uint8_t node_id);

            /**
             * @brief Build combined JointState message for all motors
             * @return Populated JointState message
             */
            JointState build_joint_state_msg();

            /**
             * @brief Build PDO statistics message
             * @return Populated PDOStatistics message
             */
            PDOStatistics build_pdo_statistics();

            /**
             * @brief Build diagnostics message for all motors
             * @return Populated DiagnosticArray message
             */
            DiagnosticArray build_diagnostics_msg();

            // =========================================================================
            // Helper Methods - State Queries
            // =========================================================================

            /**
             * @brief Check if motor is in operational state
             * @param node_id Motor node ID
             * @return true if motor is enabled and operational
             */
            bool is_motor_operational(uint8_t node_id) const;

            /**
             * @brief Get CIA402 state machine state as string
             * @param node_id Motor node ID
             * @return State name (e.g., "OPERATION_ENABLED")
             */
            std::string get_motor_state_string(uint8_t node_id) const;

            /**
             * @brief Get motor instance by node_id
             * @param node_id Motor node ID
             * @return Pointer to MotorInstance or nullptr if not found
             */
            MotorInstance* get_motor(uint8_t node_id);

            /**
             * @brief Get motor instance by node_id (const version)
             * @param node_id Motor node ID
             * @return Pointer to MotorInstance or nullptr if not found
             */
            const MotorInstance* get_motor(uint8_t node_id) const;
    };

}  // namespace ros2_waveshare
