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

#include "ros2_waveshare/odometry_publisher_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace ros2_waveshare {

    OdometryPublisherNode::OdometryPublisherNode(const rclcpp::NodeOptions& options)
        : Node("odometry_publisher", options),
        x_(0.0),
        y_(0.0),
        theta_(0.0),
        linear_velocity_(0.0),
        angular_velocity_(0.0),
        last_position_rad_(0.0),
        first_feedback_(true) {
        // Declare and get parameters
        this->declare_parameter("wheel_radius", 0.1);
        this->declare_parameter("publish_rate", 50.0);
        this->declare_parameter("motor_namespace", "/motors/motor_1");
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("publish_tf", true);
        this->declare_parameter("initial_x", 0.0);
        this->declare_parameter("initial_y", 0.0);
        this->declare_parameter("initial_theta", 0.0);

        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        motor_namespace_ = this->get_parameter("motor_namespace").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        initial_x_ = this->get_parameter("initial_x").as_double();
        initial_y_ = this->get_parameter("initial_y").as_double();
        initial_theta_ = this->get_parameter("initial_theta").as_double();

        // Initialize pose
        x_ = initial_x_;
        y_ = initial_y_;
        theta_ = initial_theta_;

        // Validate parameters
        if (wheel_radius_ <= 0.0) {
            RCLCPP_ERROR(this->get_logger(), "wheel_radius must be positive, got: %.3f",
                wheel_radius_);
            throw std::runtime_error("Invalid wheel_radius parameter");
        }
        if (publish_rate_ <= 0.0) {
            RCLCPP_ERROR(this->get_logger(), "publish_rate must be positive, got: %.3f",
                publish_rate_);
            throw std::runtime_error("Invalid publish_rate parameter");
        }

        RCLCPP_INFO(this->get_logger(), "Odometry parameters: wheel_radius=%.3f m, rate=%.1f Hz",
            wheel_radius_, publish_rate_);
        RCLCPP_INFO(this->get_logger(), "Frames: odom='%s', base='%s'",
            odom_frame_.c_str(), base_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Motor feedback topic: '%s/feedback'",
            motor_namespace_.c_str());

        // Create subscriber for motor feedback
        std::string feedback_topic = motor_namespace_ + "/feedback";
        feedback_sub_ = this->create_subscription<ros2_waveshare_msgs::msg::MotorFeedback>(
            feedback_topic,
            10,
            std::bind(&OdometryPublisherNode::feedback_callback, this, std::placeholders::_1));

        // Create odometry publisher
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // Create TF broadcaster
        if (publish_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        }

        // Create publishing timer
        auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);
        publish_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
            std::bind(&OdometryPublisherNode::publish_callback, this));

        RCLCPP_INFO(this->get_logger(), "OdometryPublisherNode initialized");
    }

    void OdometryPublisherNode::feedback_callback(
        const ros2_waveshare_msgs::msg::MotorFeedback::SharedPtr msg) {
        rclcpp::Time current_time = this->now();

        if (first_feedback_) {
            // Initialize on first feedback
            last_position_rad_ = msg->position_rad;
            last_update_time_ = current_time;
            first_feedback_ = false;
            RCLCPP_INFO(this->get_logger(), "First feedback received - odometry initialized");
            return;
        }

        // Calculate time delta
        double dt = (current_time - last_update_time_).seconds();
        if (dt <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "Non-positive time delta: %.6f s", dt);
            return;
        }

        // Integrate odometry from encoder data
        integrate_odometry(msg->position_rad, msg->velocity_rad_s, dt);

        // Update state
        last_position_rad_ = msg->position_rad;
        last_update_time_ = current_time;
    }

    void OdometryPublisherNode::integrate_odometry(
        double position_rad,
        double velocity_rad_s,
        double dt) {
        // Convert angular velocity to linear velocity
        // v = ω * r
        linear_velocity_ = velocity_rad_s * wheel_radius_;

        // For single motor, motion is always along x-axis (no rotation)
        // Position integration using trapezoidal rule
        double delta_position_rad = position_rad - last_position_rad_;
        double delta_distance = delta_position_rad * wheel_radius_;

        // Update position along x-axis
        // For single motor: always moving in x direction, no y or theta change
        x_ += delta_distance * std::cos(theta_);
        y_ += delta_distance * std::sin(theta_);
        // theta_ remains constant for single motor (always 0)

        // Angular velocity is always 0 for single motor
        angular_velocity_ = 0.0;

        RCLCPP_DEBUG(this->get_logger(),
            "Odom: x=%.3f m, v=%.3f m/s, delta=%.4f m",
            x_, linear_velocity_, delta_distance);
    }

    void OdometryPublisherNode::publish_callback() {
        if (first_feedback_) {
            // Don't publish until we receive first feedback
            return;
        }

        publish_odometry();

        if (publish_tf_) {
            publish_tf();
        }
    }

    void OdometryPublisherNode::publish_odometry() {
        auto odom_msg = nav_msgs::msg::Odometry();

        // Header
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = odom_frame_;
        odom_msg.child_frame_id = base_frame_;

        // Pose
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = create_quaternion_from_yaw(theta_);

        // Pose covariance (simplified - can be tuned)
        // Row-major 6x6 matrix (x, y, z, rotation about X, Y, Z)
        odom_msg.pose.covariance[0] = 0.01; // x
        odom_msg.pose.covariance[7] = 0.01; // y
        odom_msg.pose.covariance[14] = 1e6; // z (not used)
        odom_msg.pose.covariance[21] = 1e6; // rotation about x (not used)
        odom_msg.pose.covariance[28] = 1e6; // rotation about y (not used)
        odom_msg.pose.covariance[35] = 0.05; // rotation about z (yaw)

        // Velocity (in base_link frame)
        odom_msg.twist.twist.linear.x = linear_velocity_;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = angular_velocity_;

        // Twist covariance
        odom_msg.twist.covariance[0] = 0.01; // linear x
        odom_msg.twist.covariance[7] = 0.01; // linear y
        odom_msg.twist.covariance[14] = 1e6; // linear z (not used)
        odom_msg.twist.covariance[21] = 1e6; // angular x (not used)
        odom_msg.twist.covariance[28] = 1e6; // angular y (not used)
        odom_msg.twist.covariance[35] = 0.05; // angular z

        odom_pub_->publish(odom_msg);
    }

    void OdometryPublisherNode::publish_tf() {
        geometry_msgs::msg::TransformStamped transform;

        // Header
        transform.header.stamp = this->now();
        transform.header.frame_id = odom_frame_;
        transform.child_frame_id = base_frame_;

        // Translation
        transform.transform.translation.x = x_;
        transform.transform.translation.y = y_;
        transform.transform.translation.z = 0.0;

        // Rotation
        transform.transform.rotation = create_quaternion_from_yaw(theta_);

        tf_broadcaster_->sendTransform(transform);
    }

    geometry_msgs::msg::Quaternion OdometryPublisherNode::create_quaternion_from_yaw(double yaw) {
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);

        geometry_msgs::msg::Quaternion quat_msg;
        quat_msg.x = q.x();
        quat_msg.y = q.y();
        quat_msg.z = q.z();
        quat_msg.w = q.w();

        return quat_msg;
    }

}  // namespace ros2_waveshare
