/**
 * @file motor_driver_action_helpers.hpp
 * @brief Inline helper methods for ROS2 action server implementations
 * @author GitHub Copilot
 * @date 2025-11-12
 *
 * Template helper methods for reducing code duplication in action handlers.
 */

#pragma once

#include "ros2_waveshare/motor_driver_node.hpp"
#include "canopen/cia402_constants.hpp"

namespace ros2_waveshare {

// =============================================================================
// Template Implementations - Action Support (Header-only)
// =============================================================================

    template<typename ActionT>
    bool MotorDriverNode::should_abort_action(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT> >& goal_handle,
        const rclcpp::Time& start_time,
        const rclcpp::Duration& timeout_duration,
        const std::string& action_name) {

        // Check for cancellation
        if (goal_handle->is_canceling()) {
            auto result = std::make_shared<typename ActionT::Result>();
            result->success = false;
            result->message = action_name + " cancelled by client";
            result->elapsed_time_sec = (this->now() - start_time).seconds();
            goal_handle->canceled(result);
            RCLCPP_WARN(this->get_logger(), "%s", result->message.c_str());
            return true;
        }

        // Check timeout
        if ((this->now() - start_time) > timeout_duration) {
            auto result = std::make_shared<typename ActionT::Result>();
            result->success = false;
            result->message = action_name + " timeout after " +
                std::to_string((this->now() - start_time).seconds()) + "s";
            result->elapsed_time_sec = (this->now() - start_time).seconds();
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
            return true;
        }

        return false;
    }

    template<typename FeedbackT>
    void MotorDriverNode::update_fsm_feedback(
        MotorInstance* motor,
        std::shared_ptr<FeedbackT> feedback,
        bool force_read) {

        auto fsm = motor->get_fsm();
        auto current_state = fsm->get_current_state(force_read);

        feedback->current_state = canopen::cia402::get_state_description(current_state);
        feedback->statusword = fsm->get_statusword();
    }

    template<typename FeedbackT, typename GoalHandleT>
    bool MotorDriverNode::execute_state_transition(
        uint8_t node_id,
        MotorInstance* motor,
        std::shared_ptr<FeedbackT> feedback,
        const std::shared_ptr<GoalHandleT>& goal_handle,
        const std::string& transition_name,
        float progress_percent,
        std::function<bool(std::shared_ptr<canopen::CIA402FSM>)> fsm_function) {

        auto fsm = motor->get_fsm();

        // Update feedback with transition info
        feedback->transition_name = transition_name;
        feedback->progress_percent = progress_percent;
        goal_handle->publish_feedback(feedback);

        RCLCPP_DEBUG(this->get_logger(),
            "Motor %d: %s (%.0f%%)", node_id, transition_name.c_str(), progress_percent);

        // Execute FSM function (convert raw pointer to shared_ptr temporarily for call)
        bool success = fsm_function(std::shared_ptr<canopen::CIA402FSM>(fsm, [](auto*){
            }));                                                                         // Non-owning shared_ptr

        if (success) {
            // Update feedback with new state
            update_fsm_feedback(motor, feedback, true);
            RCLCPP_INFO(this->get_logger(),
                "Motor %d %s successful: %s (statusword=0x%04X)",
                node_id, transition_name.c_str(),
                feedback->current_state.c_str(), feedback->statusword);
        } else {
            RCLCPP_ERROR(this->get_logger(),
                "Motor %d %s failed", node_id, transition_name.c_str());
        }

        return success;
    }

}  // namespace ros2_waveshare
