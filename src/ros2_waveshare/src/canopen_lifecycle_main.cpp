/**
 * @file canopen_lifecycle_main.cpp
 * @brief Main entry point for the CANopen lifecycle node
 * @author effibot (andrea.efficace1@gmail.com)
 * @date 2025-10-22
 */

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "ros2_microphase/canopen_lifecycle.hpp"

int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create executor
    rclcpp::executors::SingleThreadedExecutor executor;

    // Create lifecycle node
    auto node = std::make_shared<ros2_microphase::CanopenLifeCycleNode>(
        rclcpp::NodeOptions());

    // Add node to executor
    executor.add_node(node->get_node_base_interface());

    // Spin executor
    executor.spin();

    // Shutdown
    rclcpp::shutdown();

    return 0;
}
