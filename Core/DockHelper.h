//
// Created by roboticslab on 6/2/23.
//

#ifndef CREATE3_CONTROLLER_DOCKHELPER_H
#define CREATE3_CONTROLLER_DOCKHELPER_H


#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "irobot_create_msgs/action/dock_servo.hpp"
#include "irobot_create_msgs/action/undock.hpp"

class DockServoClientNode : public rclcpp::Node
{
public:
    using DockServo = irobot_create_msgs::action::DockServo;
    using GoalHandleDockServo = rclcpp_action::ClientGoalHandle<DockServo>;

    explicit DockServoClientNode(const std::string& action_name)
            : Node("dock_servo_client_node")
    {
        // Create an action client
        action_client_ = rclcpp_action::create_client<DockServo>(this, action_name);

        // Wait for the action server to become available
        while (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the action server.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Action server not available, waiting...");
        }

    }


    void send_goal()
    {
        auto goal_msg = DockServo::Goal();
//        goal_msg.dock = true;  // Set the dock command to true

        auto send_goal_options = rclcpp_action::Client<DockServo>::SendGoalOptions();
        send_goal_options.goal_response_callback =
                std::bind(&DockServoClientNode::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.result_callback =
                std::bind(&DockServoClientNode::result_callback, this, std::placeholders::_1);
        goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);
    }
private:
    void goal_response_callback(std::shared_future<GoalHandleDockServo::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the action server.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Goal accepted by the action server.");
    }

    void result_callback(const GoalHandleDockServo::WrappedResult& result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Action completed successfully.");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_INFO(this->get_logger(), "Action was aborted.");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(this->get_logger(), "Action was canceled.");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
                return;
        }
    }

    rclcpp_action::Client<DockServo>::SharedPtr action_client_;
    std::shared_future<GoalHandleDockServo::SharedPtr> goal_handle_;
};


// undocking


class UndockClientNode : public rclcpp::Node
{
public:
    using Undock = irobot_create_msgs::action::Undock;
    using GoalHandleUndock = rclcpp_action::ClientGoalHandle<Undock>;
    explicit UndockClientNode(const std::string& action_name)
            : Node("undock_client_node")
    {
        // Create an action client
        action_client_ = rclcpp_action::create_client<Undock>(this, action_name);

        // Wait for the action server to become available
        while (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the action server.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Action server not available, waiting...");
        }

        // Send a goal to the action server
//        send_goal();
    }


    void send_goal()
    {
        auto goal_msg = Undock::Goal();
//        goal_msg.undock = true;  // Set the undock command to true

        auto send_goal_options = rclcpp_action::Client<Undock>::SendGoalOptions();
        send_goal_options.goal_response_callback =
                std::bind(&UndockClientNode::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.result_callback =
                std::bind(&UndockClientNode::result_callback, this, std::placeholders::_1);
        goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);
    }
private:
    void goal_response_callback(std::shared_future<GoalHandleUndock::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the action server.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Goal accepted by the action server.");
    }

    void result_callback(const GoalHandleUndock::WrappedResult& result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Action completed successfully.");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_INFO(this->get_logger(), "Action was aborted.");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(this->get_logger(), "Action was canceled.");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
                return;
        }
    }

    rclcpp_action::Client<Undock>::SharedPtr action_client_;
    std::shared_future<GoalHandleUndock::SharedPtr> goal_handle_;
};


#endif //CREATE3_CONTROLLER_DOCKHELPER_H
