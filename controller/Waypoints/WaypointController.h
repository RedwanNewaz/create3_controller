//
// Created by redwan on 6/24/23.
//

#ifndef CREATE3_CONTROLLER_WAYPOINTCONTROLLER_H
#define CREATE3_CONTROLLER_WAYPOINTCONTROLLER_H
#include <functional>
#include <memory>
#include <thread>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "action_waypoints_interfaces/action/waypoints.hpp"
#include "rapidcsv.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
    #define ACTION_TUTORIALS_CPP_EXPORT __attribute__ ((dllexport))
    #define ACTION_TUTORIALS_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define ACTION_TUTORIALS_CPP_EXPORT __declspec(dllexport)
    #define ACTION_TUTORIALS_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef ACTION_TUTORIALS_CPP_BUILDING_DLL
    #define ACTION_TUTORIALS_CPP_PUBLIC ACTION_TUTORIALS_CPP_EXPORT
  #else
    #define ACTION_TUTORIALS_CPP_PUBLIC ACTION_TUTORIALS_CPP_IMPORT
  #endif
  #define ACTION_TUTORIALS_CPP_PUBLIC_TYPE ACTION_TUTORIALS_CPP_PUBLIC
  #define ACTION_TUTORIALS_CPP_LOCAL
#else
#define ACTION_TUTORIALS_CPP_EXPORT __attribute__ ((visibility("default")))
#define ACTION_TUTORIALS_CPP_IMPORT
#if __GNUC__ >= 4
#define ACTION_TUTORIALS_CPP_PUBLIC __attribute__ ((visibility("default")))
#define ACTION_TUTORIALS_CPP_LOCAL  __attribute__ ((visibility("hidden")))
#else
#define ACTION_TUTORIALS_CPP_PUBLIC
    #define ACTION_TUTORIALS_CPP_LOCAL
#endif
#define ACTION_TUTORIALS_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

namespace navigation {
    struct point {
        double x;
        double y;

        double operator -(const point&other) const
        {
            double dx = this->x - other.x;
            double dy = this->y - other.y;

            return std::sqrt(dx * dx + dy * dy);
        }
    };

    class WaypointController : public rclcpp::Node {
    public:
        using Waypoints = action_waypoints_interfaces::action::Waypoints;
        using GoalHandleWaypoints = rclcpp_action::ServerGoalHandle<Waypoints>;
        ACTION_TUTORIALS_CPP_PUBLIC
        explicit WaypointController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    private:
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_pose_;
        rclcpp_action::Server<Waypoints>::SharedPtr action_server_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr obs_sub_;
        double m_resolution; //m
        double m_updateFq; //Hz
        point m_robot;
        bool m_odom_init;


    protected:
        rclcpp_action::GoalResponse handle_goal(
                const rclcpp_action::GoalUUID & uuid,
                std::shared_ptr<const Waypoints::Goal> goal);

        rclcpp_action::CancelResponse handle_cancel(
                const std::shared_ptr<GoalHandleWaypoints> goal_handle);

        void handle_accepted(const std::shared_ptr<GoalHandleWaypoints> goal_handle);

        void execute(const std::shared_ptr<GoalHandleWaypoints> goal_handle);

        std::vector<point> interpolateWaypoints(const std::vector<point>& waypoints, double resolution);

        void pubMsg(double x, double y);
    };
}


#endif //CREATE3_CONTROLLER_WAYPOINTCONTROLLER_H
