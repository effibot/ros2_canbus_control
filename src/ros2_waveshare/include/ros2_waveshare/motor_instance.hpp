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

#ifndef ROS2_WAVESHARE__MOTOR_INSTANCE_HPP_
#define ROS2_WAVESHARE__MOTOR_INSTANCE_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// ROS2 messages
#include <ros2_waveshare_msgs/msg/motor_feedback.hpp>
#include <ros2_waveshare_msgs/msg/motor_command.hpp>
#include <ros2_waveshare_msgs/msg/motor_status.hpp>

// ROS2 services
#include <ros2_waveshare_msgs/srv/sdo_read.hpp>
#include <ros2_waveshare_msgs/srv/sdo_write.hpp>
#include <ros2_waveshare_msgs/srv/set_operation_mode.hpp>
#include <ros2_waveshare_msgs/srv/get_motor_info.hpp>

// ROS2 actions
#include <ros2_waveshare_msgs/action/enable_motor.hpp>
#include <ros2_waveshare_msgs/action/reset_fault.hpp>
#include <ros2_waveshare_msgs/action/move_to_position.hpp>

// Waveshare CANopen library
#include <canopen/sdo_client.hpp>
#include <canopen/cia402_fsm.hpp>
#include <canopen/cia402_constants.hpp>
#include <canopen/object_dictionary.hpp>

namespace ros2_waveshare {

/**
 * @brief Encapsulates all state and ROS2 interfaces for a single CANopen motor
 *
 * This class manages:
 * - CANopen components (SDOClient, CIA402FSM, ObjectDictionary)
 * - Unit conversion parameters (encoder counts, gear ratio, torque constant)
 * - State tracking (initialized, enabled, latest feedback)
 * - ROS2 publishers, subscribers, services, and action servers
 * - Thread-safe access to motor feedback data
 */
    class MotorInstance {
        public:
            using MotorFeedback = ros2_waveshare_msgs::msg::MotorFeedback;
            using MotorCommand = ros2_waveshare_msgs::msg::MotorCommand;
            using MotorStatus = ros2_waveshare_msgs::msg::MotorStatus;

            using SDORead = ros2_waveshare_msgs::srv::SDORead;
            using SDOWrite = ros2_waveshare_msgs::srv::SDOWrite;
            using SetOperationMode = ros2_waveshare_msgs::srv::SetOperationMode;
            using GetMotorInfo = ros2_waveshare_msgs::srv::GetMotorInfo;

            using EnableMotor = ros2_waveshare_msgs::action::EnableMotor;
            using ResetFault = ros2_waveshare_msgs::action::ResetFault;
            using MoveToPosition = ros2_waveshare_msgs::action::MoveToPosition;

            /**
             * @brief Constructor
             * @param node_id CANopen node ID (1-127)
             * @param name Motor name (e.g., "traction_motor_left")
             * @param type Motor type (e.g., "traction", "steering")
             */
            MotorInstance(uint8_t node_id, const std::string& name, const std::string& type);

            /**
             * @brief Destructor
             */
            ~MotorInstance() = default;

            // =========================================================================
            // Motor Identification
            // =========================================================================

            uint8_t get_node_id() const { return node_id_; }
            const std::string& get_name() const { return name_; }
            const std::string& get_type() const { return type_; }

            // =========================================================================
            // CANopen Components Access
            // =========================================================================

            canopen::SDOClient* get_sdo_client() { return sdo_client_.get(); }
            const canopen::SDOClient* get_sdo_client() const { return sdo_client_.get(); }

            canopen::CIA402FSM* get_fsm() { return fsm_.get(); }
            const canopen::CIA402FSM* get_fsm() const { return fsm_.get(); }

            canopen::ObjectDictionary& get_dictionary() {
                if (!dictionary_) {
                    throw std::runtime_error("Object dictionary not initialized for motor: " +
                        name_);
                }
                return *dictionary_;
            }
            const canopen::ObjectDictionary& get_dictionary() const {
                if (!dictionary_) {
                    throw std::runtime_error("Object dictionary not initialized for motor: " +
                        name_);
                }
                return *dictionary_;
            }

            bool has_dictionary() const { return dictionary_.has_value(); }

            void set_sdo_client(std::unique_ptr<canopen::SDOClient> client) {
                sdo_client_ = std::move(client);
            }

            void set_fsm(std::unique_ptr<canopen::CIA402FSM> fsm) {
                fsm_ = std::move(fsm);
            }

            void set_dictionary(const canopen::ObjectDictionary& dict) {
                dictionary_ = dict;
            }

            // =========================================================================
            // Unit Conversion Parameters
            // =========================================================================

            void set_conversion_params(double counts_per_rev, double gear_ratio,
                double torque_const) {
                counts_per_revolution_ = counts_per_rev;
                gear_ratio_ = gear_ratio;
                torque_constant_nm_per_a_ = torque_const;
            }

            double get_counts_per_revolution() const { return counts_per_revolution_; }
            double get_gear_ratio() const { return gear_ratio_; }
            double get_torque_constant() const { return torque_constant_nm_per_a_; }

            /**
             * @brief Convert encoder counts to radians (output shaft)
             */
            double counts_to_rad(int32_t counts) const {
                return (2.0 * M_PI * counts) / (counts_per_revolution_ * gear_ratio_);
            }

            /**
             * @brief Convert radians (output shaft) to encoder counts
             */
            int32_t rad_to_counts(double rad) const {
                return static_cast<int32_t>((rad * counts_per_revolution_ * gear_ratio_) /
                       (2.0 * M_PI));
            }

            /**
             * @brief Convert encoder counts/sec to rad/sec (output shaft)
             */
            double counts_per_sec_to_rad_per_sec(int32_t counts_per_sec) const {
                return (2.0 * M_PI * counts_per_sec) / (counts_per_revolution_ * gear_ratio_);
            }

            /**
             * @brief Convert rad/sec (output shaft) to encoder counts/sec
             */
            int32_t rad_per_sec_to_counts_per_sec(double rad_per_sec) const {
                return static_cast<int32_t>((rad_per_sec * counts_per_revolution_ * gear_ratio_) /
                       (2.0 * M_PI));
            }

            /**
             * @brief Convert motor current to output torque
             */
            double current_to_torque(int16_t current_ma) const {
                double current_a = current_ma / 1000.0;
                return current_a * torque_constant_nm_per_a_ * gear_ratio_;
            }

            /**
             * @brief Convert output torque to motor current
             */
            int16_t torque_to_current(double torque_nm) const {
                double current_a = torque_nm / (torque_constant_nm_per_a_ * gear_ratio_);
                return static_cast<int16_t>(current_a * 1000.0);
            }

            // =========================================================================
            // Control Limits
            // =========================================================================

            void set_limits(double max_velocity_rad_s, double max_torque_nm,
                int32_t max_current_ma) {
                max_velocity_rad_s_ = max_velocity_rad_s;
                max_torque_nm_ = max_torque_nm;
                max_current_ma_ = max_current_ma;
            }

            double get_max_velocity() const { return max_velocity_rad_s_; }
            double get_max_torque() const { return max_torque_nm_; }
            int32_t get_max_current() const { return max_current_ma_; }

            // =========================================================================
            // State Tracking
            // =========================================================================

            bool is_initialized() const { return initialized_.load(); }
            void set_initialized(bool value) { initialized_.store(value); }

            bool is_enabled() const { return enabled_.load(); }
            void set_enabled(bool value) { enabled_.store(value); }

            // =========================================================================
            // Feedback Data (Thread-safe)
            // =========================================================================

            /**
             * @brief Get latest motor feedback (thread-safe copy)
             */
            MotorFeedback get_latest_feedback() const {
                std::lock_guard<std::mutex> lock(feedback_mutex_);
                return latest_feedback_;
            }

            /**
             * @brief Update latest motor feedback (thread-safe)
             */
            void update_latest_feedback(const MotorFeedback& feedback) {
                std::lock_guard<std::mutex> lock(feedback_mutex_);
                latest_feedback_ = feedback;
            }

            /**
             * @brief Update TPDO receive timestamp and counter
             */
            void record_tpdo1_received() {
                last_tpdo1_time_ = std::chrono::steady_clock::now();
                tpdo1_count_.fetch_add(1);
            }

            void record_tpdo2_received() {
                last_tpdo2_time_ = std::chrono::steady_clock::now();
                tpdo2_count_.fetch_add(1);
            }

            void increment_tpdo_missed_count() {
                tpdo_missed_count_.fetch_add(1);
            }

            std::chrono::steady_clock::time_point get_last_tpdo1_time() const {
                return last_tpdo1_time_;
            }

            std::chrono::steady_clock::time_point get_last_tpdo2_time() const {
                return last_tpdo2_time_;
            }

            uint32_t get_tpdo1_count() const { return tpdo1_count_.load(); }
            uint32_t get_tpdo2_count() const { return tpdo2_count_.load(); }
            uint32_t get_tpdo_missed_count() const { return tpdo_missed_count_.load(); }

            // =========================================================================
            // Motor State Updates (from PDO data)
            // =========================================================================

            /**
             * @brief Update statusword from TPDO1
             */
            void update_statusword(uint16_t statusword) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                statusword_ = statusword;
            }

            /**
             * @brief Get current statusword
             */
            uint16_t get_statusword() const {
                std::lock_guard<std::mutex> lock(state_mutex_);
                return statusword_;
            }

            /**
             * @brief Update position from TPDO1 (encoder counts)
             */
            void update_position(int32_t position_counts) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                position_counts_ = position_counts;
            }

            /**
             * @brief Get current position (encoder counts)
             */
            int32_t get_position() const {
                std::lock_guard<std::mutex> lock(state_mutex_);
                return position_counts_;
            }

            /**
             * @brief Update velocity from TPDO2 (counts/sec)
             */
            void update_velocity(int32_t velocity_counts_per_sec) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                velocity_counts_per_sec_ = velocity_counts_per_sec;
            }

            /**
             * @brief Get current velocity (counts/sec)
             */
            int32_t get_velocity() const {
                std::lock_guard<std::mutex> lock(state_mutex_);
                return velocity_counts_per_sec_;
            }

            /**
             * @brief Update current from TPDO2 (milliamps)
             */
            void update_current(int16_t current_ma) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                current_ma_ = current_ma;
            }

            /**
             * @brief Get current (milliamps)
             */
            int16_t get_current() const {
                std::lock_guard<std::mutex> lock(state_mutex_);
                return current_ma_;
            }

            /**
             * @brief Update operation mode
             */
            void update_operation_mode(int8_t mode) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                operation_mode_ = mode;
            }

            /**
             * @brief Get current operation mode
             */
            int8_t get_operation_mode() const {
                std::lock_guard<std::mutex> lock(state_mutex_);
                return operation_mode_;
            }

            /**
             * @brief Update last TPDO1 receive time
             */
            void update_last_tpdo1_time(const rclcpp::Time& time) {
                last_tpdo1_ros_time_ = time;
            }

            /**
             * @brief Update last TPDO2 receive time
             */
            void update_last_tpdo2_time(const rclcpp::Time& time) {
                last_tpdo2_ros_time_ = time;
            }

            /**
             * @brief Get previous state for change detection
             */
            canopen::cia402::State get_previous_state() const {
                std::lock_guard<std::mutex> lock(state_mutex_);
                return previous_state_;
            }

            /**
             * @brief Set previous state for change detection
             */
            void set_previous_state(canopen::cia402::State state) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                previous_state_ = state;
            }

            // =========================================================================
            // ROS2 Publishers
            // =========================================================================

            void set_feedback_publisher(rclcpp::Publisher<MotorFeedback>::SharedPtr pub) {
                pub_feedback_ = pub;
            }

            void set_status_publisher(rclcpp::Publisher<MotorStatus>::SharedPtr pub) {
                pub_status_ = pub;
            }

            rclcpp::Publisher<MotorFeedback>::SharedPtr get_feedback_publisher() {
                return pub_feedback_;
            }

            rclcpp::Publisher<MotorStatus>::SharedPtr get_status_publisher() {
                return pub_status_;
            }

            // =========================================================================
            // ROS2 Subscribers
            // =========================================================================

            void set_command_subscriber(rclcpp::Subscription<MotorCommand>::SharedPtr sub) {
                sub_command_ = sub;
            }

            // =========================================================================
            // ROS2 Services
            // =========================================================================

            void set_sdo_read_service(rclcpp::Service<SDORead>::SharedPtr srv) {
                srv_sdo_read_ = srv;
            }

            void set_sdo_write_service(rclcpp::Service<SDOWrite>::SharedPtr srv) {
                srv_sdo_write_ = srv;
            }

            void set_set_mode_service(rclcpp::Service<SetOperationMode>::SharedPtr srv) {
                srv_set_mode_ = srv;
            }

            void set_get_info_service(rclcpp::Service<GetMotorInfo>::SharedPtr srv) {
                srv_get_info_ = srv;
            }

            // =========================================================================
            // ROS2 Action Servers
            // =========================================================================

            void set_enable_action_server(rclcpp_action::Server<EnableMotor>::SharedPtr action) {
                action_enable_ = action;
            }

            void set_reset_fault_action_server(
                rclcpp_action::Server<ResetFault>::SharedPtr action) {
                action_reset_fault_ = action;
            }

            void set_move_action_server(rclcpp_action::Server<MoveToPosition>::SharedPtr action) {
                action_move_ = action;
            }

        private:
            // Motor identification
            uint8_t node_id_;
            std::string name_;
            std::string type_;

            // CANopen components (per motor)
            std::unique_ptr<canopen::SDOClient> sdo_client_;
            std::unique_ptr<canopen::CIA402FSM> fsm_;
            std::optional<canopen::ObjectDictionary> dictionary_;

            // Unit conversion parameters
            double counts_per_revolution_{10000.0};
            double gear_ratio_{1.0};
            double torque_constant_nm_per_a_{0.05};

            // Control limits
            double max_velocity_rad_s_{10.0};
            double max_torque_nm_{5.0};
            int32_t max_current_ma_{10000};

            // State tracking
            std::atomic<bool> initialized_{false};
            std::atomic<bool> enabled_{false};

            // Motor state data (protected by mutex)
            mutable std::mutex state_mutex_;
            uint16_t statusword_{0};
            int32_t position_counts_{0};
            int32_t velocity_counts_per_sec_{0};
            int16_t current_ma_{0};
            int8_t operation_mode_{0};
            canopen::cia402::State previous_state_{canopen::cia402::State::NOT_READY_TO_SWITCH_ON};
            rclcpp::Time last_tpdo1_ros_time_;
            rclcpp::Time last_tpdo2_ros_time_;

            // Latest feedback data (protected by mutex)
            mutable std::mutex feedback_mutex_;
            MotorFeedback latest_feedback_;

            // TPDO receive statistics
            std::chrono::steady_clock::time_point last_tpdo1_time_;
            std::chrono::steady_clock::time_point last_tpdo2_time_;
            std::atomic<uint32_t> tpdo1_count_{0};
            std::atomic<uint32_t> tpdo2_count_{0};
            std::atomic<uint32_t> tpdo_missed_count_{0};

            // ROS2 Publishers (per motor)
            rclcpp::Publisher<MotorFeedback>::SharedPtr pub_feedback_;
            rclcpp::Publisher<MotorStatus>::SharedPtr pub_status_;

            // ROS2 Subscribers (per motor)
            rclcpp::Subscription<MotorCommand>::SharedPtr sub_command_;

            // ROS2 Services (per motor)
            rclcpp::Service<SDORead>::SharedPtr srv_sdo_read_;
            rclcpp::Service<SDOWrite>::SharedPtr srv_sdo_write_;
            rclcpp::Service<SetOperationMode>::SharedPtr srv_set_mode_;
            rclcpp::Service<GetMotorInfo>::SharedPtr srv_get_info_;

            // ROS2 Action Servers (per motor)
            rclcpp_action::Server<EnableMotor>::SharedPtr action_enable_;
            rclcpp_action::Server<ResetFault>::SharedPtr action_reset_fault_;
            rclcpp_action::Server<MoveToPosition>::SharedPtr action_move_;
    };

}  // namespace ros2_waveshare

#endif  // ROS2_WAVESHARE__MOTOR_INSTANCE_HPP_
