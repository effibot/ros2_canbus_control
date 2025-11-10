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

#include <chrono>
#include <functional>
#include <stdexcept>
#include <cmath>
#include <fstream>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace ros2_waveshare {

// =============================================================================
// Constructor & Destructor
// =============================================================================

    MotorDriverNode::MotorDriverNode(const rclcpp::NodeOptions& options)
        : Node("motor_driver", options) {
        RCLCPP_INFO(this->get_logger(), "Initializing MotorDriverNode...");

        try {
            // Step 1: Load all parameters
            load_parameters();

            // Step 2: Initialize CAN socket
            initialize_can_socket();

            // Step 3: Initialize all motors
            initialize_motors();

            // Step 4: Initialize PDO manager
            initialize_pdo_manager();

            // Step 5: Setup combined publishers
            setup_publishers();

            // Step 6: Start timers
            setup_timers();

            RCLCPP_INFO(this->get_logger(),
                "MotorDriverNode initialized successfully with %zu motors",
                motors_.size());

        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(),
                "Failed to initialize MotorDriverNode: %s", e.what());
            throw;
        }
    }

    MotorDriverNode::~MotorDriverNode() {
        RCLCPP_INFO(this->get_logger(), "Shutting down MotorDriverNode...");

        // Stop all timers
        if (timer_sync_) timer_sync_->cancel();
        if (timer_joint_state_) timer_joint_state_->cancel();
        if (timer_diagnostics_) timer_diagnostics_->cancel();

        // Disable all motors
        for (auto& [node_id, motor] : motors_) {
            if (motor->is_enabled() && motor->get_fsm()) {
                try {
                    RCLCPP_INFO(this->get_logger(),
                        "Disabling motor %d (%s)...", node_id, motor->get_name().c_str());
                    motor->get_fsm()->disable_operation();
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(),
                        "Failed to disable motor %d: %s", node_id, e.what());
                }
            }
        }

        // Stop PDO manager
        if (pdo_manager_) {
            pdo_manager_.reset();
        }

        // Close CAN socket
        if (can_socket_) {
            can_socket_.reset();
        }

        RCLCPP_INFO(this->get_logger(), "MotorDriverNode shutdown complete");
    }

// =============================================================================
// Parameter Loading
// =============================================================================

    void MotorDriverNode::load_parameters() {
        RCLCPP_INFO(this->get_logger(), "Loading parameters...");

        // CAN interface
        can_interface_ = this->declare_parameter<std::string>("can_interface", "can0");

        // PDO configuration
        sync_rate_hz_ = this->declare_parameter<double>("pdo.sync_rate_hz", 100.0);
        sync_enabled_ = this->declare_parameter<bool>("pdo.sync_enabled", true);
        tpdo_timeout_ms_ = this->declare_parameter<int>("pdo.tpdo_timeout_ms", 100);

        // Publishing rates
        joint_state_rate_hz_ = this->declare_parameter<double>("publishing.joint_state_rate_hz",
            100.0);
        pdo_statistics_rate_hz_ =
            this->declare_parameter<double>("publishing.pdo_statistics_rate_hz", 1.0);
        diagnostics_rate_hz_ = this->declare_parameter<double>("publishing.diagnostics_rate_hz",
            1.0);

        // Timeouts
        sdo_timeout_ms_ = this->declare_parameter<int>("timeouts.sdo_timeout_ms", 1000);
        state_transition_timeout_ms_ =
            this->declare_parameter<int>("timeouts.state_transition_timeout_ms", 1000);
        action_default_timeout_sec_ =
            this->declare_parameter<double>("timeouts.action_default_timeout_sec", 5.0);

        // Safety
        enable_watchdog_ = this->declare_parameter<bool>("safety.enable_watchdog", true);
        watchdog_timeout_ms_ = this->declare_parameter<int>("safety.watchdog_timeout_ms", 500);
        auto_disable_on_fault_ = this->declare_parameter<bool>("safety.auto_disable_on_fault",
            true);

        // Diagnostics thresholds
        temperature_warning_c_ =
            this->declare_parameter<double>("diagnostics.temperature_warning_c", 60.0);
        temperature_error_c_ = this->declare_parameter<double>("diagnostics.temperature_error_c",
            80.0);
        voltage_min_warning_v_ =
            this->declare_parameter<double>("diagnostics.voltage_min_warning_v", 22.0);
        voltage_max_warning_v_ =
            this->declare_parameter<double>("diagnostics.voltage_max_warning_v", 28.0);

        // Logging
        log_sdo_transactions_ = this->declare_parameter<bool>("logging.log_sdo_transactions",
            false);
        log_pdo_statistics_ = this->declare_parameter<bool>("logging.log_pdo_statistics", true);
        log_state_transitions_ = this->declare_parameter<bool>("logging.log_state_transitions",
            true);

        RCLCPP_INFO(this->get_logger(),
            "Parameters loaded: CAN=%s, SYNC=%.1fHz, JointState=%.1fHz",
            can_interface_.c_str(), sync_rate_hz_, joint_state_rate_hz_);
    }

// =============================================================================
// CAN Socket Initialization
// =============================================================================

    void MotorDriverNode::initialize_can_socket() {
        RCLCPP_INFO(this->get_logger(),
            "Initializing CAN socket on interface: %s", can_interface_.c_str());

        try {
            can_socket_ = std::make_shared<waveshare::RealCANSocket>(can_interface_, 100); // 100ms timeout

            RCLCPP_INFO(this->get_logger(), "CAN socket opened successfully");

        } catch (const std::exception& e) {
            throw std::runtime_error(
                std::string("Failed to open CAN interface ") + can_interface_ +
                ": " + e.what() +
                "\nMake sure the interface is up: sudo ip link set " + can_interface_ +
                " up type can bitrate 500000");
        }
    }

// =============================================================================
// Motor Initialization
// =============================================================================

    void MotorDriverNode::initialize_motors() {
        RCLCPP_INFO(this->get_logger(), "Initializing motors...");

        // Get list of motor parameter namespaces
        // Expected structure: motors.motor_1.node_id, motors.motor_2.node_id, etc.

        const std::vector<std::string> motor_keys = {"motor_1", "motor_2", "motor_3", "motor_4"};

        for (const auto& key : motor_keys) {
            std::string prefix = "motors." + key;

            // Check if this motor is configured
            if (!this->has_parameter(prefix + ".node_id")) {
                // Try to declare it to see if it exists in parameter file
                try {
                    this->declare_parameter<int>(prefix + ".node_id", 0);
                } catch (...) {
                    // Parameter doesn't exist, skip this motor
                    continue;
                }
            }

            uint8_t node_id = static_cast<uint8_t>(this->get_parameter(prefix +
                ".node_id").as_int());

            if (node_id == 0) {
                // Motor not configured (node_id 0 is invalid)
                continue;
            }

            RCLCPP_INFO(this->get_logger(), "Configuring %s (node_id=%d)...", key.c_str(), node_id);

            // Load motor parameters
            std::string motor_name = this->declare_parameter<std::string>(prefix + ".name",
                "motor_" + std::to_string(node_id));
            std::string motor_type = this->declare_parameter<std::string>(prefix + ".type",
                "unknown");
            std::string config_file = this->declare_parameter<std::string>(prefix + ".config_file",
                "");

            double counts_per_rev = this->declare_parameter<double>(prefix +
                ".counts_per_revolution", 10000.0);
            double gear_ratio = this->declare_parameter<double>(prefix + ".gear_ratio", 1.0);
            double torque_const = this->declare_parameter<double>(prefix +
                ".torque_constant_nm_per_a", 0.05);

            double max_velocity = this->declare_parameter<double>(prefix + ".max_velocity_rad_s",
                10.0);
            double max_torque = this->declare_parameter<double>(prefix + ".max_torque_nm", 5.0);
            int max_current = this->declare_parameter<int>(prefix + ".max_current_ma", 10000);

            // Create motor instance
            auto motor = std::make_unique<MotorInstance>(node_id, motor_name, motor_type);

            // Set conversion parameters
            motor->set_conversion_params(counts_per_rev, gear_ratio, torque_const);
            motor->set_limits(max_velocity, max_torque, max_current);

            // Load object dictionary
            if (!config_file.empty()) {
                try {
                    auto dict = canopen::ObjectDictionary(config_file);
                    motor->set_dictionary(dict);
                    RCLCPP_INFO(this->get_logger(),
                        "Loaded object dictionary from: %s", config_file.c_str());
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(),
                        "Failed to load object dictionary from %s: %s",
                        config_file.c_str(), e.what());
                }
            }

            // Initialize CANopen components
            // Note: SDOClient requires dictionary, so skip if dictionary not loaded
            if (!motor->has_dictionary()) {
                RCLCPP_ERROR(this->get_logger(),
                    "Cannot initialize motor %s: object dictionary not loaded", motor_name.c_str());
                continue;
            }

            auto sdo_client = std::make_unique<canopen::SDOClient>(
                can_socket_, motor->get_dictionary(), node_id);

            auto fsm = std::make_unique<canopen::CIA402FSM>(
                *sdo_client, motor->get_dictionary());

            motor->set_sdo_client(std::move(sdo_client));
            motor->set_fsm(std::move(fsm));

            // Setup ROS2 publishers
            std::string ns = "/motors/" + key;

            auto pub_feedback = this->create_publisher<MotorFeedback>(
                ns + "/feedback", rclcpp::QoS(10));
            motor->set_feedback_publisher(pub_feedback);

            auto pub_status = this->create_publisher<MotorStatus>(
                ns + "/status", rclcpp::QoS(10));
            motor->set_status_publisher(pub_status);

            // Setup ROS2 subscribers
            auto sub_command = this->create_subscription<MotorCommand>(
                ns + "/command",
                rclcpp::QoS(10),
                [this, node_id](const MotorCommand::SharedPtr msg) {
                    this->on_motor_command(node_id, msg);
                });
            motor->set_command_subscriber(sub_command);

            // Setup ROS2 services
            auto srv_sdo_read = this->create_service<SDORead>(
                ns + "/sdo/read",
                [this, node_id](const std::shared_ptr<SDORead::Request> req,
                std::shared_ptr<SDORead::Response> res) {
                    this->handle_sdo_read(node_id, req, res);
                });
            motor->set_sdo_read_service(srv_sdo_read);

            auto srv_sdo_write = this->create_service<SDOWrite>(
                ns + "/sdo/write",
                [this, node_id](const std::shared_ptr<SDOWrite::Request> req,
                std::shared_ptr<SDOWrite::Response> res) {
                    this->handle_sdo_write(node_id, req, res);
                });
            motor->set_sdo_write_service(srv_sdo_write);

            auto srv_set_mode = this->create_service<SetOperationMode>(
                ns + "/set_operation_mode",
                [this, node_id](const std::shared_ptr<SetOperationMode::Request> req,
                std::shared_ptr<SetOperationMode::Response> res) {
                    this->handle_set_operation_mode(node_id, req, res);
                });
            motor->set_set_mode_service(srv_set_mode);

            auto srv_get_info = this->create_service<GetMotorInfo>(
                ns + "/get_motor_info",
                [this, node_id](const std::shared_ptr<GetMotorInfo::Request> req,
                std::shared_ptr<GetMotorInfo::Response> res) {
                    this->handle_get_motor_info(node_id, req, res);
                });
            motor->set_get_info_service(srv_get_info);

            // Setup ROS2 action servers
            auto action_enable = rclcpp_action::create_server<EnableMotor>(
                this,
                ns + "/enable",
                [this, node_id](const rclcpp_action::GoalUUID& uuid,
                std::shared_ptr<const EnableMotor::Goal> goal) {
                    return this->handle_enable_goal(node_id, uuid, goal);
                },
                [this,
                node_id](const std::shared_ptr<rclcpp_action::ServerGoalHandle<EnableMotor> >
                goal_handle) {
                    return this->handle_enable_cancel(node_id, goal_handle);
                },
                [this,
                node_id](const std::shared_ptr<rclcpp_action::ServerGoalHandle<EnableMotor> >
                goal_handle) {
                    this->handle_enable_accepted(node_id, goal_handle);
                });
            motor->set_enable_action_server(action_enable);

            auto action_reset_fault = rclcpp_action::create_server<ResetFault>(
                this,
                ns + "/reset_fault",
                [this, node_id](const rclcpp_action::GoalUUID& uuid,
                std::shared_ptr<const ResetFault::Goal> goal) {
                    return this->handle_reset_fault_goal(node_id, uuid, goal);
                },
                [this,
                node_id](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetFault> >
                goal_handle) {
                    return this->handle_reset_fault_cancel(node_id, goal_handle);
                },
                [this,
                node_id](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ResetFault> >
                goal_handle) {
                    this->handle_reset_fault_accepted(node_id, goal_handle);
                });
            motor->set_reset_fault_action_server(action_reset_fault);

            auto action_move = rclcpp_action::create_server<MoveToPosition>(
                this,
                ns + "/move_to_position",
                [this, node_id](const rclcpp_action::GoalUUID& uuid,
                std::shared_ptr<const MoveToPosition::Goal> goal) {
                    return this->handle_move_goal(node_id, uuid, goal);
                },
                [this,
                node_id](const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPosition> >
                goal_handle) {
                    return this->handle_move_cancel(node_id, goal_handle);
                },
                [this,
                node_id](const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPosition> >
                goal_handle) {
                    this->handle_move_accepted(node_id, goal_handle);
                });
            motor->set_move_action_server(action_move);

            motor->set_initialized(true);

            RCLCPP_INFO(this->get_logger(),
                "Motor %d (%s) initialized: type=%s, counts/rev=%.0f, gear=%.1f",
                node_id, motor->get_name().c_str(), motor->get_type().c_str(),
                motor->get_counts_per_revolution(), motor->get_gear_ratio());

            // Add to motors map
            motors_[node_id] = std::move(motor);
        }

        if (motors_.empty()) {
            throw std::runtime_error("No motors configured! Check parameter file.");
        }

        RCLCPP_INFO(this->get_logger(), "Initialized %zu motors", motors_.size());
    }

// =============================================================================
// Helper Methods
// =============================================================================

    MotorInstance* MotorDriverNode::get_motor(uint8_t node_id) {
        auto it = motors_.find(node_id);
        if (it != motors_.end()) {
            return it->second.get();
        }
        return nullptr;
    }

    const MotorInstance* MotorDriverNode::get_motor(uint8_t node_id) const {
        auto it = motors_.find(node_id);
        if (it != motors_.end()) {
            return it->second.get();
        }
        return nullptr;
    }

}  // namespace ros2_waveshare
