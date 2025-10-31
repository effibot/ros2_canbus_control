/**
 * @file canopen_lifecycle.hpp
 * @author effibot (andrea.efficace1@gmail.com)
 * @brief ROS2 Lifecycle Node to integrate ros2_canopen with waveshare_cpp SocketCAN bridge
 * @version 0.1
 * @date 2025-10-22
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

// # ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
using rclcpp_lifecycle::LifecycleNode;
using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
// # waveshare_cpp includes
#include "../../lib/waveshare_cpp/include/waveshare.hpp"
using namespace waveshare;

namespace ros2_waveshare {
    /**
     * @brief ROS2 Lifecycle node that manages the SocketCAN Bridge
     *  defined in waveshare_cpp library.
     * In its core, this node is "just" a wrapper around the already
     * fully working and independent SocketCANBridge class.
     * It exposes lifecycle transitions to start/stop the bridge and diagnostic
     * to check the status of the node.
     *
     * @see `waveshare_cpp/scripts/wave_bridgecpp` for a non-ROS2 example on how to use the bridge.
     *
     * @note To be able to use the bridge, three objects must be used:
     *
     * - a `BridgeConfig` object to initialize the bridge settings.
     *
     * - a `usb_to_socketcan_callback()` function to handle received CAN frames from the USB adapter.
     *
     * - a `socketcan_to_usb_callback()` function to handle received CAN frames from the SocketCAN interface.
     *
     * The LifeCycle states are:
     *
     * - UNCONFIGURED -> INACTIVE: Load params from yaml file, create the BridgeConfig object.
     *
     * - INACTIVE -> ACTIVE: Actually create the SocketCanBridge instance and
     * open a socket to the loaded socketCAN interface.
     *
     * - ACTIVE -> INACTIVE: Destroy the SocketCanBridge instance and close the socket.
     *
     * - INACTIVE -> UNCONFIGURED: Reset all the parameters and prepare for a new configuration.
     *
     * - Any state -> SHUTDOWN: Clean up all resources.
     *
     */
    class CanopenLifeCycleNode : public LifecycleNode {
        public:
            /**
             * @brief Construct a new Canopen Life Cycle Node object
             *
             * @param options Node options
             */
            explicit CanopenLifeCycleNode(
                const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

            /**
             * @brief Destroy the Canopen Life Cycle Node object
             *
             */
            ~CanopenLifeCycleNode();

            // === Callbacks for lifecycle transitions ===
            LifecycleNodeInterface::CallbackReturn on_configure(
                const rclcpp_lifecycle::State& previous_state) override;

            LifecycleNodeInterface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State& previous_state) override;

            LifecycleNodeInterface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State& previous_state) override;

            LifecycleNodeInterface::CallbackReturn on_cleanup(
                const rclcpp_lifecycle::State& previous_state) override;

            LifecycleNodeInterface::CallbackReturn on_shutdown(
                const rclcpp_lifecycle::State& previous_state) override;

        private:
            // Pointer to the Bridge configuration
            std::shared_ptr<BridgeConfig> bridge_config_;

            // Pointer to the SocketCAN Bridge
            std::shared_ptr<SocketCANBridge> socketcan_bridge_;

            // === Diagnostic ===

            // Diagnostic Updater
            diagnostic_updater::Updater diag_updater_;

            void setup_diagnostics();

            // Diagnostic Publisher for the SocketCAN Bridge
            std::shared_ptr<diagnostic_updater::TopicDiagnostic> bridge_diag_;

            // Diagnostic Callback
            void bridge_diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper& stat);



            // <<< Bridge Callbacks >>>

            /**
             * @brief Callback function to handle CAN frames received from the USB adapter
             *
             * @param usb_frame Received Waveshare translated CAN frame
             * @param socketcan_frame Corresponding SocketCAN frame to be forwarded
             */
            void usb_to_socketcan_callback_impl(const VariableFrame& usb_frame,
                const ::can_frame& socketcan_frame);

            /**
             * @brief Callback function to handle CAN frames received from the SocketCAN interface
             *
             * @param socket_frame Received SocketCAN frame over the socket
             * @param usb_frame Corresponding Waveshare CAN frame to write to the USB adapter
             * that will be translated and sent over the CAN bus.
             */
            void socketcan_to_usb_callback_impl(const ::can_frame& socketcan_frame,
                const VariableFrame& usb_frame);


    };


}