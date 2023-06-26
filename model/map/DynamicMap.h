//
// Created by redwan on 6/25/23.
//

#ifndef CREATE3_CONTROLLER_DYNAMICMAP_H
#define CREATE3_CONTROLLER_DYNAMICMAP_H
#include <functional>
#include <memory>
#include <thread>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>
#include "action_waypoints_interfaces/action/dynmap.hpp"

namespace map_server
{
    using namespace std::placeholders;
    class DynamicMap : public rclcpp::Node  {
    public:
        using Dynmap = action_waypoints_interfaces::action::Dynmap;
        using GoalHandleDynmap = rclcpp_action::ServerGoalHandle<Dynmap>;
        explicit DynamicMap(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    private:
        rclcpp_action::Server<Dynmap>::SharedPtr action_server_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
        double m_resolution;
        double m_offsetX, m_offsetY;
    protected:
        rclcpp_action::GoalResponse handle_goal(
                const rclcpp_action::GoalUUID & uuid,
                std::shared_ptr<const Dynmap::Goal> goal);

        rclcpp_action::CancelResponse handle_cancel(
                const std::shared_ptr<GoalHandleDynmap> goal_handle);

        void handle_accepted(const std::shared_ptr<GoalHandleDynmap> goal_handle);

        void execute(const std::shared_ptr<GoalHandleDynmap> goal_handle);

    };

}

#endif //CREATE3_CONTROLLER_DYNAMICMAP_H
