//
// Created by roboticslab on 11/22/22.
//

#ifndef CREATE3_CONTROLLER_DWA_PLANNER_ROS_H
#define CREATE3_CONTROLLER_DWA_PLANNER_ROS_H

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "create3_controller/state_estimator.h"
#include "create3_controller/dwa_planner.h"




class dwa_planner_ros: public rclcpp::Node, public DynamicWindow::planner{
public:
    explicit dwa_planner_ros(StatePtr stateEstimator);

protected:
    void timer_callback();
    void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);
    void rviz_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void obstacle_callback(geometry_msgs::msg::PoseArray::SharedPtr msg);

protected:
    void publish_cmd(double v, double w);
    void publish_short_horizon_traj(Traj& traj);
    void publish_obstacles();




private:
    bool initialized_;
    StatePtr stateEstimator_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr traj_pub_, obs_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obs_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_;
    DynamicWindow::Config config_;
    Control control_;
    Obstacle obstacles_;
    tf2::Transform goal_pose_;
    std::mutex mu_;
    std::once_flag obs_flag_;
};


#endif //CREATE3_CONTROLLER_DWA_PLANNER_ROS_H
