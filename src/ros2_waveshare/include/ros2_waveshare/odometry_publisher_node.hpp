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

#ifndef ROS2_WAVESHARE__ODOMETRY_PUBLISHER_NODE_HPP_
#define ROS2_WAVESHARE__ODOMETRY_PUBLISHER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ros2_waveshare_msgs/msg/motor_feedback.hpp>

namespace ros2_waveshare {

/**
 * @brief Odometry publisher node for single motor
 *
 * Integrates encoder feedback from a single motor to compute odometry.
 * For single motor testing, this provides 1D motion along the x-axis.
 *
 * Features:
 * - Encoder position integration
 * - Velocity calculation from encoder feedback
 * - nav_msgs/Odometry message publishing
 * - TF2 transform broadcasting (odom → base_link)
 * - Configurable wheel radius and frame names
 *
 * Topics:
 * - Subscribes: /motors/motor_X/feedback (MotorFeedback)
 * - Publishes: /odom (Odometry)
 * - Publishes: /tf (TF transforms)
 */
    class OdometryPublisherNode : public rclcpp::Node {
        public:
            /**
             * @brief Constructor
             *
             * Initializes ROS2 node, declares parameters, sets up subscribers,
             * publishers, and TF broadcaster
             */
            explicit OdometryPublisherNode(
                const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

            /**
             * @brief Destructor
             */
            ~OdometryPublisherNode() = default;

        private:
            /**
             * @brief Callback for motor feedback messages
             *
             * Receives encoder data, integrates position, and updates odometry
             *
             * @param msg Motor feedback message containing encoder data
             */
            void feedback_callback(const ros2_waveshare_msgs::msg::MotorFeedback::SharedPtr msg);

            /**
             * @brief Publishing timer callback
             *
             * Publishes odometry and TF at configured rate (default 50 Hz)
             */
            void publish_callback();

            /**
             * @brief Integrate encoder position to update odometry
             *
             * Performs numerical integration of encoder velocity to compute
             * linear displacement along x-axis
             *
             * @param position_rad Current encoder position in radians
             * @param velocity_rad_s Current encoder velocity in rad/s
             * @param dt Time step since last update
             */
            void integrate_odometry(double position_rad, double velocity_rad_s, double dt);

            /**
             * @brief Publish odometry message
             *
             * Populates and publishes nav_msgs/Odometry with current pose and velocity
             */
            void publish_odometry();

            /**
             * @brief Publish TF transform
             *
             * Broadcasts transform from odom_frame to base_frame
             */
            void publish_tf();

            /**
             * @brief Create quaternion from yaw angle
             *
             * @param yaw Yaw angle in radians
             * @return Quaternion representing rotation around z-axis
             */
            geometry_msgs::msg::Quaternion create_quaternion_from_yaw(double yaw);

            // ROS2 interfaces
            rclcpp::Subscription<ros2_waveshare_msgs::msg::MotorFeedback>::SharedPtr feedback_sub_;
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
            rclcpp::TimerBase::SharedPtr publish_timer_;
            std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

            // Odometry state
            double x_;          ///< Position along x-axis (m)
            double y_;          ///< Position along y-axis (m) - always 0 for single motor
            double theta_;      ///< Orientation (rad) - always 0 for single motor
            double linear_velocity_; ///< Linear velocity along x-axis (m/s)
            double angular_velocity_; ///< Angular velocity (rad/s) - always 0 for single motor

            // Encoder state
            double last_position_rad_; ///< Previous encoder position for integration
            rclcpp::Time last_update_time_; ///< Timestamp of last feedback message
            bool first_feedback_; ///< Flag for first feedback message

            // Parameters
            double wheel_radius_; ///< Wheel radius in meters
            double publish_rate_; ///< Publishing frequency (Hz)
            std::string motor_namespace_; ///< Namespace for motor feedback topic
            std::string base_frame_; ///< Base link frame name
            std::string odom_frame_; ///< Odometry frame name
            bool publish_tf_;   ///< Whether to publish TF transforms

            // Initial pose
            double initial_x_;
            double initial_y_;
            double initial_theta_;
    };

}  // namespace ros2_waveshare

#endif  // ROS2_WAVESHARE__ODOMETRY_PUBLISHER_NODE_HPP_
