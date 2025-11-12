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

#include "ros2_waveshare/plugins/single_motor_model.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>
#include <sstream>
#include <iomanip>

namespace ros2_waveshare {

    SingleMotorModel::SingleMotorModel()
        : target_velocity_rad_s_(0.0),
        current_velocity_rad_s_(0.0),
        error_(0.0),
        integral_error_(0.0),
        previous_error_(0.0),
        previous_output_(0.0),
        kp_(1.0),
        ki_(0.1),
        kd_(0.05),
        max_velocity_(1.0),
        max_acceleration_(2.0),
        max_integral_(10.0),
        wheel_radius_(0.1),
        gear_ratio_(1.0),
        initialized_(false) {
    }

    bool SingleMotorModel::initialize(
        rclcpp::Node::SharedPtr node,
        const std::string& param_namespace) {
        node_ = node;

        // Declare and get parameters
        std::string ns = param_namespace.empty() ? "" : param_namespace + ".";

        node_->declare_parameter(ns + "kp", kp_);
        node_->declare_parameter(ns + "ki", ki_);
        node_->declare_parameter(ns + "kd", kd_);
        node_->declare_parameter(ns + "max_velocity", max_velocity_);
        node_->declare_parameter(ns + "max_acceleration", max_acceleration_);
        node_->declare_parameter(ns + "max_integral", max_integral_);
        node_->declare_parameter(ns + "wheel_radius", wheel_radius_);
        node_->declare_parameter(ns + "gear_ratio", gear_ratio_);

        kp_ = node_->get_parameter(ns + "kp").as_double();
        ki_ = node_->get_parameter(ns + "ki").as_double();
        kd_ = node_->get_parameter(ns + "kd").as_double();
        max_velocity_ = node_->get_parameter(ns + "max_velocity").as_double();
        max_acceleration_ = node_->get_parameter(ns + "max_acceleration").as_double();
        max_integral_ = node_->get_parameter(ns + "max_integral").as_double();
        wheel_radius_ = node_->get_parameter(ns + "wheel_radius").as_double();
        gear_ratio_ = node_->get_parameter(ns + "gear_ratio").as_double();

        // Validate parameters
        if (wheel_radius_ <= 0.0) {
            RCLCPP_ERROR(node_->get_logger(),
                "SingleMotorModel: wheel_radius must be positive, got: %.3f",
                wheel_radius_);
            return false;
        }

        if (gear_ratio_ <= 0.0) {
            RCLCPP_ERROR(node_->get_logger(),
                "SingleMotorModel: gear_ratio must be positive, got: %.3f",
                gear_ratio_);
            return false;
        }

        RCLCPP_INFO(node_->get_logger(),
            "SingleMotorModel initialized:");
        RCLCPP_INFO(node_->get_logger(),
            "  PID gains: kp=%.3f, ki=%.3f, kd=%.3f", kp_, ki_, kd_);
        RCLCPP_INFO(node_->get_logger(),
            "  Limits: max_vel=%.2f m/s, max_accel=%.2f m/s², max_integral=%.2f",
            max_velocity_, max_acceleration_, max_integral_);
        RCLCPP_INFO(node_->get_logger(),
            "  Physical: wheel_radius=%.3f m, gear_ratio=%.2f",
            wheel_radius_, gear_ratio_);

        // Reset state
        reset();
        initialized_ = true;

        return true;
    }

    void SingleMotorModel::update(
        const geometry_msgs::msg::Twist& cmd_vel,
        const ros2_waveshare_msgs::msg::MotorFeedback& feedback,
        double dt) {
        if (!initialized_) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                "SingleMotorModel: update() called before initialize()");
            return;
        }

        // Extract target linear velocity from cmd_vel (only use x component)
        double target_linear_velocity = cmd_vel.linear.x;

        // Clamp target to max velocity
        target_linear_velocity = clamp(target_linear_velocity, -max_velocity_, max_velocity_);

        // Convert linear velocity to angular velocity
        // ω = v / r
        target_velocity_rad_s_ = target_linear_velocity / wheel_radius_;

        // Apply gear ratio if needed (motor side vs wheel side)
        target_velocity_rad_s_ *= gear_ratio_;

        // Get current velocity from feedback
        current_velocity_rad_s_ = feedback.velocity_rad_s;

        // Calculate error
        error_ = target_velocity_rad_s_ - current_velocity_rad_s_;

        // Update integral with anti-windup
        integral_error_ += error_ * dt;
        integral_error_ = clamp(integral_error_, -max_integral_, max_integral_);

        RCLCPP_DEBUG(node_->get_logger(),
            "Update: target=%.3f rad/s, current=%.3f rad/s, error=%.3f rad/s, integral=%.3f",
            target_velocity_rad_s_, current_velocity_rad_s_, error_, integral_error_);
    }

    ros2_waveshare_msgs::msg::MotorCommand SingleMotorModel::compute_control() {
        if (!initialized_) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                "SingleMotorModel: compute_control() called before initialize()");
            auto cmd = ros2_waveshare_msgs::msg::MotorCommand();
            cmd.target_velocity_rad_s = 0.0;
            cmd.operation_mode = ros2_waveshare_msgs::msg::MotorCommand::PROFILE_VELOCITY;
            return cmd;
        }

        // PID calculation
        double derivative = 0.0;
        if (previous_error_ != 0.0) { // Avoid derivative kick on first iteration
            derivative = error_ - previous_error_;
        }

        double pid_output = kp_ * error_ + ki_ * integral_error_ + kd_ * derivative;

        // Apply acceleration limiting
        double max_change = max_acceleration_ * wheel_radius_; // Convert to rad/s change
        double output_change = pid_output - previous_output_;
        if (std::abs(output_change) > max_change) {
            pid_output = previous_output_ + (output_change > 0.0 ? max_change : -max_change);
        }

        // Store for next iteration
        previous_error_ = error_;
        previous_output_ = pid_output;

        // Create motor command
        auto cmd = ros2_waveshare_msgs::msg::MotorCommand();
        cmd.target_velocity_rad_s = pid_output;
        cmd.operation_mode = ros2_waveshare_msgs::msg::MotorCommand::PROFILE_VELOCITY;

        RCLCPP_DEBUG(node_->get_logger(),
            "Control: pid_output=%.3f rad/s, derivative=%.3f",
            pid_output, derivative);

        return cmd;
    }

    void SingleMotorModel::reset() {
        target_velocity_rad_s_ = 0.0;
        current_velocity_rad_s_ = 0.0;
        error_ = 0.0;
        integral_error_ = 0.0;
        previous_error_ = 0.0;
        previous_output_ = 0.0;

        if (initialized_) {
            RCLCPP_INFO(node_->get_logger(), "SingleMotorModel: State reset");
        }
    }

    ros2_waveshare_msgs::msg::MotorCommand SingleMotorModel::shutdown() {
        RCLCPP_INFO(node_->get_logger(), "SingleMotorModel: Shutdown - sending zero velocity");

        reset();

        auto cmd = ros2_waveshare_msgs::msg::MotorCommand();
        cmd.target_velocity_rad_s = 0.0;
        cmd.operation_mode = ros2_waveshare_msgs::msg::MotorCommand::PROFILE_VELOCITY;

        return cmd;
    }

    std::string SingleMotorModel::get_state_string() const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3);
        oss << "SingleMotorModel State:\n";
        oss << "  Target: " << target_velocity_rad_s_ << " rad/s\n";
        oss << "  Current: " << current_velocity_rad_s_ << " rad/s\n";
        oss << "  Error: " << error_ << " rad/s\n";
        oss << "  Integral: " << integral_error_ << "\n";
        oss << "  Previous Error: " << previous_error_ << " rad/s\n";
        oss << "  PID Output: " << previous_output_ << " rad/s\n";
        oss << "  Gains: kp=" << kp_ << ", ki=" << ki_ << ", kd=" << kd_;

        return oss.str();
    }

    double SingleMotorModel::clamp(double value, double min_val, double max_val) const {
        if (value < min_val) return min_val;
        if (value > max_val) return max_val;
        return value;
    }

}  // namespace ros2_waveshare

// Register this plugin with pluginlib
// This macro makes the plugin discoverable by the pluginlib system
PLUGINLIB_EXPORT_CLASS(ros2_waveshare::SingleMotorModel, ros2_waveshare::DynamicModelInterface)
