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

#include "ros2_waveshare/velocity_converter_node.hpp"

namespace ros2_waveshare {

    VelocityConverterNode::VelocityConverterNode(const rclcpp::NodeOptions& options)
        : Node("velocity_converter", options),
        feedback_received_(false),
        plugin_initialized_(false),
        control_loop_count_(0),
        cmd_vel_count_(0),
        feedback_count_(0) {
        // Declare parameters with defaults
        this->declare_parameter("plugin_name", "ros2_waveshare::SingleMotorModel");
        this->declare_parameter("control_rate", 100.0);
        this->declare_parameter("cmd_vel_timeout", 0.5);
        this->declare_parameter("motor_namespace", "/motors/motor_1");
        this->declare_parameter("node_id", 1);

        // Get parameters
        plugin_name_ = this->get_parameter("plugin_name").as_string();
        control_rate_ = this->get_parameter("control_rate").as_double();
        cmd_vel_timeout_ = this->get_parameter("cmd_vel_timeout").as_double();
        motor_namespace_ = this->get_parameter("motor_namespace").as_string();
        node_id_ = static_cast<uint8_t>(this->get_parameter("node_id").as_int());

        RCLCPP_INFO(this->get_logger(), "VelocityConverterNode initializing...");
        RCLCPP_INFO(this->get_logger(), "  Plugin: %s", plugin_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Control rate: %.1f Hz", control_rate_);
        RCLCPP_INFO(this->get_logger(), "  Cmd vel timeout: %.2f s", cmd_vel_timeout_);
        RCLCPP_INFO(this->get_logger(), "  Motor namespace: %s", motor_namespace_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Node ID: %d", node_id_);

        // Initialize pluginlib class loader
        try {
            plugin_loader_ = std::make_shared<pluginlib::ClassLoader<DynamicModelInterface> >(
                "ros2_waveshare", "ros2_waveshare::DynamicModelInterface");
            RCLCPP_INFO(this->get_logger(), "Pluginlib ClassLoader created successfully");
        } catch (const pluginlib::PluginlibException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create plugin loader: %s", ex.what());
            throw;
        }

        // Load the plugin
        if (!load_plugin(plugin_name_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load plugin on startup");
            throw std::runtime_error("Plugin loading failed");
        }

        // Create subscribers
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&VelocityConverterNode::cmd_vel_callback, this, std::placeholders::_1));

        feedback_sub_ = this->create_subscription<ros2_waveshare_msgs::msg::MotorFeedback>(
            motor_namespace_ + "/feedback", 10,
            std::bind(&VelocityConverterNode::feedback_callback, this, std::placeholders::_1));

        // Create publisher
        motor_cmd_pub_ = this->create_publisher<ros2_waveshare_msgs::msg::MotorCommand>(
            motor_namespace_ + "/command", 10);

        // Create service
        reload_plugin_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "~/reload_plugin",
            std::bind(&VelocityConverterNode::reload_plugin_callback, this,
            std::placeholders::_1, std::placeholders::_2));

        // Create control timer
        auto timer_period = std::chrono::duration<double>(1.0 / control_rate_);
        control_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
            std::bind(&VelocityConverterNode::control_timer_callback, this));

        // Initialize time tracking
        last_cmd_vel_time_ = this->now();
        last_control_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "VelocityConverterNode initialized successfully");
        RCLCPP_INFO(this->get_logger(), "Waiting for cmd_vel and motor feedback...");
    }

    VelocityConverterNode::~VelocityConverterNode() {
        RCLCPP_INFO(this->get_logger(), "VelocityConverterNode shutting down");
        RCLCPP_INFO(this->get_logger(), "Statistics:");
        RCLCPP_INFO(this->get_logger(), "  Control loops: %zu", control_loop_count_);
        RCLCPP_INFO(this->get_logger(), "  Cmd vel messages: %zu", cmd_vel_count_);
        RCLCPP_INFO(this->get_logger(), "  Feedback messages: %zu", feedback_count_);

        unload_plugin();
    }

    bool VelocityConverterNode::load_plugin(const std::string& plugin_name) {
        RCLCPP_INFO(this->get_logger(), "Loading plugin: %s", plugin_name.c_str());

        // Unload existing plugin if any
        if (dynamic_model_) {
            unload_plugin();
        }

        try {
            // Create plugin instance
            dynamic_model_ = plugin_loader_->createSharedInstance(plugin_name);
            RCLCPP_INFO(this->get_logger(), "Plugin instance created");

            // Initialize plugin with this node's handle
            if (!dynamic_model_->initialize(this->shared_from_this(), "single_motor_model")) {
                RCLCPP_ERROR(this->get_logger(), "Plugin initialization failed");
                dynamic_model_.reset();
                plugin_initialized_ = false;
                return false;
            }

            plugin_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Plugin loaded and initialized successfully");
            return true;

        } catch (const pluginlib::PluginlibException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load plugin: %s", ex.what());
            dynamic_model_.reset();
            plugin_initialized_ = false;
            return false;
        }
    }

    void VelocityConverterNode::unload_plugin() {
        if (dynamic_model_) {
            RCLCPP_INFO(this->get_logger(), "Unloading plugin");
            dynamic_model_->shutdown();
            dynamic_model_.reset();
            plugin_initialized_ = false;
        }
    }

    void VelocityConverterNode::reload_plugin_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Reload plugin service called");

        // Get current plugin name from parameters (might have changed)
        plugin_name_ = this->get_parameter("plugin_name").as_string();

        if (load_plugin(plugin_name_)) {
            response->success = true;
            response->message = "Plugin reloaded successfully: " + plugin_name_;
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        } else {
            response->success = false;
            response->message = "Failed to reload plugin: " + plugin_name_;
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    void VelocityConverterNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        current_cmd_vel_ = msg;
        last_cmd_vel_time_ = this->now();
        cmd_vel_count_++;

        RCLCPP_DEBUG(this->get_logger(), "Received cmd_vel: linear.x=%.3f, angular.z=%.3f",
            msg->linear.x, msg->angular.z);
    }

    void VelocityConverterNode::feedback_callback(
        const ros2_waveshare_msgs::msg::MotorFeedback::SharedPtr msg) {
        current_feedback_ = msg;
        feedback_received_ = true;
        feedback_count_++;

        RCLCPP_DEBUG(this->get_logger(), "Received feedback: position=%.3f, velocity=%.3f",
            msg->position_rad, msg->velocity_rad_s);
    }

    void VelocityConverterNode::control_timer_callback() {
        control_loop_count_++;

        // Check if plugin is ready
        if (!plugin_initialized_ || !dynamic_model_) {
            if (control_loop_count_ % 100 == 0) { // Log every second at 100 Hz
                RCLCPP_WARN(this->get_logger(), "Plugin not initialized, skipping control loop");
            }
            return;
        }

        // Check if we have feedback
        if (!feedback_received_ || !current_feedback_) {
            if (control_loop_count_ % 100 == 0) {
                RCLCPP_WARN(this->get_logger(), "No motor feedback received yet");
            }
            return;
        }

        // Check cmd_vel timeout (safety feature)
        double time_since_cmd = (this->now() - last_cmd_vel_time_).seconds();
        bool cmd_vel_valid = (current_cmd_vel_ != nullptr) && (time_since_cmd < cmd_vel_timeout_);

        geometry_msgs::msg::Twist target_velocity;
        if (cmd_vel_valid) {
            target_velocity = *current_cmd_vel_;
        } else {
            // Timeout or no command received - use zero velocity
            target_velocity.linear.x = 0.0;
            target_velocity.linear.y = 0.0;
            target_velocity.linear.z = 0.0;
            target_velocity.angular.x = 0.0;
            target_velocity.angular.y = 0.0;
            target_velocity.angular.z = 0.0;

            if (control_loop_count_ % 100 == 0 && current_cmd_vel_) {
                RCLCPP_WARN(this->get_logger(), "Cmd_vel timeout (%.2f s), using zero velocity",
                    time_since_cmd);
            }
        }

        // Calculate time since last update
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_control_time_).seconds();
        last_control_time_ = current_time;

        // Update plugin with target and current velocities
        try {
            dynamic_model_->update(target_velocity, *current_feedback_, dt);
        } catch (const std::exception& ex) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Plugin update failed: %s", ex.what());
            return;
        }

        // Compute control output
        ros2_waveshare_msgs::msg::MotorCommand cmd;
        try {
            cmd = dynamic_model_->compute_control();
        } catch (const std::exception& ex) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Plugin compute_control failed: %s", ex.what());
            return;
        }

        // Set header and node ID
        cmd.header.stamp = this->now();
        cmd.header.frame_id = "base_link";
        cmd.node_id = node_id_;

        // Publish command
        motor_cmd_pub_->publish(cmd);

        // Log state periodically (every 1 second at 100 Hz)
        if (control_loop_count_ % 100 == 0) {
            RCLCPP_INFO(this->get_logger(),
                "Control loop %zu: target_vel=%.3f m/s, current_vel=%.3f rad/s, cmd_vel=%.3f rad/s",
                control_loop_count_,
                target_velocity.linear.x,
                current_feedback_->velocity_rad_s,
                cmd.target_velocity_rad_s);

            // Log plugin state
            std::string state = dynamic_model_->get_state_string();
            RCLCPP_DEBUG(this->get_logger(), "Plugin state:\n%s", state.c_str());
        }
    }

}  // namespace ros2_waveshare

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_waveshare::VelocityConverterNode)
