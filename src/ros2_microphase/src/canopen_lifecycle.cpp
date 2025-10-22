#include "ros2_microphase/canopen_lifecycle.hpp"
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace ros2_microphase {

    CanopenLifeCycleNode::CanopenLifeCycleNode(
        const rclcpp::NodeOptions& options)
        : LifecycleNode("waveshare_bridge", options)
        , diag_updater_(this) {
        RCLCPP_INFO(get_logger(), "CANopen Lifecycle Node created");

        // Declare parameters with defaults
        declare_parameter<std::string>("socketcan_interface", "vcan0");
        declare_parameter<std::string>("usb_device", "/dev/ttyUSB0");
        declare_parameter<int>("serial_baud", 2000000); // 2Mbps
        declare_parameter<int>("can_baud", 1000000); // 1Mbps
        declare_parameter<bool>("auto_retransmit", true);
        declare_parameter<std::string>("can_mode", "normal"); // NORMAL
        declare_parameter<int>("usb_read_timeout_ms", 1000);
        declare_parameter<int>("socketcan_read_timeout_ms", 1000);
        declare_parameter<double>("diagnostics_rate", 1.0); // Hz
    }

    CanopenLifeCycleNode::~CanopenLifeCycleNode() {
        // Stop bridge if still running
        if (socketcan_bridge_) {
            try {
                socketcan_bridge_->stop();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Error stopping bridge in destructor: %s", e.what());
            }
        }
    }

    LifecycleNodeInterface::CallbackReturn CanopenLifeCycleNode::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/) {
        RCLCPP_INFO(get_logger(), "Configuring...");

        try {
            // Create bridge configuration from parameters
            bridge_config_ = std::make_shared<BridgeConfig>();

            bridge_config_->socketcan_interface = get_parameter("socketcan_interface").as_string();
            bridge_config_->usb_device_path = get_parameter("usb_device").as_string();
            bridge_config_->usb_read_timeout_ms = get_parameter("usb_read_timeout_ms").as_int();
            bridge_config_->socketcan_read_timeout_ms =
                get_parameter("socketcan_read_timeout_ms").as_int();

            // Support variable to detect if default values have been used
            bool use_default[3] = {false};
            // Map baud rates
            int serial_baud = get_parameter("serial_baud").as_int();
            bridge_config_->serial_baud_rate = serialbaud_from_int(serial_baud, use_default[0]);
            if (use_default[0])
                RCLCPP_WARN(get_logger(), "Invalid Serial baud %d, using 2Mbps", serial_baud);
            int can_baud = get_parameter("can_baud").as_int();

            bridge_config_->can_baud_rate = canbaud_from_int(can_baud, use_default[1]);
            if (use_default[1])
                RCLCPP_WARN(get_logger(), "Invalid CAN baud %d, using 1Mbps", can_baud);

            std::string can_mode = get_parameter("can_mode").as_string();
            bridge_config_->can_mode = canmode_from_string(can_mode, use_default[2]);
            if (use_default[2])
                RCLCPP_WARN(get_logger(), "Invalid CAN mode %s, using NORMAL", can_mode.c_str());

            // Map auto_retransmit - no need to convert
            bridge_config_->auto_retransmit = get_parameter("auto_retransmit").as_bool();

            // Validate configuration
            bridge_config_->validate();

            RCLCPP_INFO(get_logger(), "Configuration loaded:");
            RCLCPP_INFO(get_logger(), "  SocketCAN: %s",
                bridge_config_->socketcan_interface.c_str());
            RCLCPP_INFO(get_logger(), "  USB Device: %s", bridge_config_->usb_device_path.c_str());
            RCLCPP_INFO(get_logger(), "  Serial Baud: %d", serial_baud);
            RCLCPP_INFO(get_logger(), "  CAN Baud: %d", can_baud);

            return LifecycleNodeInterface::CallbackReturn::SUCCESS;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Configuration failed: %s", e.what());
            return LifecycleNodeInterface::CallbackReturn::FAILURE;
        }
    }

    LifecycleNodeInterface::CallbackReturn CanopenLifeCycleNode::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/) {
        RCLCPP_INFO(get_logger(), "Activating...");

        try {
            // Create bridge using factory (automatically creates vcan, opens sockets, configures USB)
            socketcan_bridge_ = SocketCANBridge::create(*bridge_config_);

            // Register callbacks for monitoring
            socketcan_bridge_->set_usb_to_socketcan_callback(
                std::bind(&CanopenLifeCycleNode::usb_to_socketcan_callback_impl, this,
                std::placeholders::_1, std::placeholders::_2));

            socketcan_bridge_->set_socketcan_to_usb_callback(
                std::bind(&CanopenLifeCycleNode::socketcan_to_usb_callback_impl, this,
                std::placeholders::_1, std::placeholders::_2));

            // Start bridge forwarding threads
            socketcan_bridge_->start();

            // Setup diagnostics
            setup_diagnostics();

            RCLCPP_INFO(get_logger(), "Bridge activated successfully");
            RCLCPP_INFO(get_logger(), "  USB interface: %s",
                socketcan_bridge_->is_usb_open() ? "OPEN" : "CLOSED");
            RCLCPP_INFO(get_logger(), "  CAN interface: %s",
                socketcan_bridge_->is_socketcan_open() ? "OPEN" : "CLOSED");

            return LifecycleNodeInterface::CallbackReturn::SUCCESS;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Activation failed: %s", e.what());
            socketcan_bridge_.reset();
            return LifecycleNodeInterface::CallbackReturn::FAILURE;
        }
    }

    LifecycleNodeInterface::CallbackReturn CanopenLifeCycleNode::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/) {
        RCLCPP_INFO(get_logger(), "Deactivating...");

        if (socketcan_bridge_) {
            try {
                // Stop bridge threads (destructor will close sockets)
                socketcan_bridge_->stop();

                // Log final statistics
                auto stats = socketcan_bridge_->get_statistics();
                RCLCPP_INFO(get_logger(), "Final Statistics:");
                RCLCPP_INFO(get_logger(), "  USB RX: %lu frames (%lu errors)",
                    stats.usb_rx_frames, stats.usb_rx_errors);
                RCLCPP_INFO(get_logger(), "  USB TX: %lu frames (%lu errors)",
                    stats.usb_tx_frames, stats.usb_tx_errors);
                RCLCPP_INFO(get_logger(), "  CAN RX: %lu frames (%lu errors)",
                    stats.socketcan_rx_frames, stats.socketcan_rx_errors);
                RCLCPP_INFO(get_logger(), "  CAN TX: %lu frames (%lu errors)",
                    stats.socketcan_tx_frames, stats.socketcan_tx_errors);
                RCLCPP_INFO(get_logger(), "  Conversion errors: %lu", stats.conversion_errors);

                socketcan_bridge_.reset();

            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Error during deactivation: %s", e.what());
            }
        }

        // Reset diagnostics
        bridge_diag_.reset();

        RCLCPP_INFO(get_logger(), "Bridge deactivated");
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn CanopenLifeCycleNode::on_cleanup(
        const rclcpp_lifecycle::State& /*previous_state*/) {
        RCLCPP_INFO(get_logger(), "Cleaning up...");

        // Reset configuration
        bridge_config_.reset();

        RCLCPP_INFO(get_logger(), "Cleanup complete");
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn CanopenLifeCycleNode::on_shutdown(
        const rclcpp_lifecycle::State& /*previous_state*/) {
        RCLCPP_INFO(get_logger(), "Shutting down...");

        if (socketcan_bridge_) {
            try {
                socketcan_bridge_->stop();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Error during shutdown: %s", e.what());
            }
            socketcan_bridge_.reset();
        }

        bridge_config_.reset();
        bridge_diag_.reset();

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void CanopenLifeCycleNode::setup_diagnostics() {
        // Setup diagnostic updater
        diag_updater_.setHardwareID("Waveshare USB-CAN-A");
        diag_updater_.add("Bridge Status", this,
            &CanopenLifeCycleNode::bridge_diagnostic_callback);

        // Set update frequency
        double rate = get_parameter("diagnostics_rate").as_double();
        diag_updater_.setPeriod(1.0 / rate);

        // Force initial update
        diag_updater_.force_update();
    }

    void CanopenLifeCycleNode::bridge_diagnostic_callback(
        diagnostic_updater::DiagnosticStatusWrapper& stat) {
        if (!socketcan_bridge_) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                "Bridge not initialized");
            return;
        }

        // Get statistics from bridge
        auto stats = socketcan_bridge_->get_statistics();

        // Check interface status
        bool usb_open = socketcan_bridge_->is_usb_open();
        bool can_open = socketcan_bridge_->is_socketcan_open();
        bool running = socketcan_bridge_->is_running();

        // Determine health status
        if (running && usb_open && can_open) {
            // Calculate error rate
            uint64_t total_frames = stats.usb_rx_frames + stats.usb_tx_frames +
                stats.socketcan_rx_frames + stats.socketcan_tx_frames;
            uint64_t total_errors = stats.usb_rx_errors + stats.usb_tx_errors +
                stats.socketcan_rx_errors + stats.socketcan_tx_errors +
                stats.conversion_errors;

            if (total_frames > 0) {
                double error_rate = static_cast<double>(total_errors) /
                    static_cast<double>(total_frames);

                if (error_rate > 0.1) {
                    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                        "High error rate detected");
                } else {
                    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                        "Bridge operational");
                }
            } else {
                stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                    "Bridge operational (no traffic)");
            }
        } else if (!running) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                "Bridge not running");
        } else if (!usb_open && !can_open) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                "Both interfaces down");
        } else {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                "One interface down");
        }

        // Add detailed status
        stat.add("USB Interface", usb_open ? "Open" : "Closed");
        stat.add("SocketCAN Interface", can_open ? "Open" : "Closed");
        stat.add("Bridge Running", running ? "Yes" : "No");

        // Add frame counters
        stat.add("USB RX Frames", stats.usb_rx_frames);
        stat.add("USB TX Frames", stats.usb_tx_frames);
        stat.add("SocketCAN RX Frames", stats.socketcan_rx_frames);
        stat.add("SocketCAN TX Frames", stats.socketcan_tx_frames);

        // Add error counters
        stat.add("USB RX Errors", stats.usb_rx_errors);
        stat.add("USB TX Errors", stats.usb_tx_errors);
        stat.add("SocketCAN RX Errors", stats.socketcan_rx_errors);
        stat.add("SocketCAN TX Errors", stats.socketcan_tx_errors);
        stat.add("Conversion Errors", stats.conversion_errors);

        // Add calculated metrics
        uint64_t total_frames = stats.usb_rx_frames + stats.usb_tx_frames +
            stats.socketcan_rx_frames + stats.socketcan_tx_frames;
        stat.add("Total Frames", total_frames);

        if (total_frames > 0) {
            uint64_t total_errors = stats.usb_rx_errors + stats.usb_tx_errors +
                stats.socketcan_rx_errors + stats.socketcan_tx_errors +
                stats.conversion_errors;
            double error_rate = (static_cast<double>(total_errors) /
                static_cast<double>(total_frames)) * 100.0;
            stat.add("Error Rate (%)", error_rate);
        }
    }

    void CanopenLifeCycleNode::usb_to_socketcan_callback_impl(
        const VariableFrame& usb_frame,
        const ::can_frame& socketcan_frame) {
        // Optional: Custom logging at DEBUG level
        RCLCPP_DEBUG(get_logger(), "USB=>CAN: ID=0x%03X, DLC=%d",
            socketcan_frame.can_id & CAN_EFF_MASK, socketcan_frame.can_dlc);

        // Update diagnostics
        diag_updater_.force_update();
    }

    void CanopenLifeCycleNode::socketcan_to_usb_callback_impl(
        const ::can_frame& socketcan_frame,
        const VariableFrame& usb_frame) {
        // Optional: Custom logging at DEBUG level
        RCLCPP_DEBUG(get_logger(), "CAN=>USB: ID=0x%03X, DLC=%d",
            socketcan_frame.can_id & CAN_EFF_MASK, socketcan_frame.can_dlc);

        // Update diagnostics
        diag_updater_.force_update();
    }

}  // namespace ros2_microphase

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_microphase::CanopenLifeCycleNode)