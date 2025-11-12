/**
 * @file motor_driver_helpers.cpp
 * @brief Non-template helper methods for motor driver node
 * @author GitHub Copilot
 * @date 2025-11-12
 */

#include "ros2_waveshare/motor_driver_node.hpp"
#include "canopen/cia402_constants.hpp"

namespace ros2_waveshare {

    bool MotorDriverNode::set_operation_mode_verified(
        uint8_t node_id,
        MotorInstance* motor,
        int8_t mode,
        std::string& error_msg) {

        auto sdo_client = motor->get_sdo_client();
        auto& dict = motor->get_dictionary();

        try {
            // Write mode
            auto mode_data = dict.to_raw(mode);
            sdo_client->write_object("modes_of_operation", mode_data);

            // Wait for mode to settle
            rclcpp::sleep_for(std::chrono::milliseconds(50));

            // Verify mode
            auto mode_display = sdo_client->read_object("modes_of_operation_display");
            int8_t actual_mode = dict.from_raw<int8_t>(mode_display);

            if (actual_mode != mode) {
                error_msg = "Failed to set operation mode: requested " +
                    std::string(canopen::cia402::get_mode_description(mode)) +
                    " (mode=" + std::to_string(mode) + ") but got " +
                    std::string(canopen::cia402::get_mode_description(actual_mode)) +
                    " (mode=" + std::to_string(actual_mode) + ")";
                return false;
            }

            RCLCPP_INFO(this->get_logger(),
                "Motor %d: Operation mode set to %s",
                node_id, canopen::cia402::get_mode_description(mode));
            return true;

        } catch (const std::exception& e) {
            error_msg = "Exception while setting operation mode: " + std::string(e.what());
            return false;
        }
    }

}  // namespace ros2_waveshare
