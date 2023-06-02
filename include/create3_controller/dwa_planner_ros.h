//
// Created by roboticslab on 11/22/22.
//

#ifndef CREATE3_CONTROLLER_DWA_PLANNER_ROS_H
#define CREATE3_CONTROLLER_DWA_PLANNER_ROS_H

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "create3_controller/dwa_planner.h"
#include "../state_estimator/JointStateEstimator.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include "utilities/param_manager2.h"
#include "../Core/ControllerManager.h"






class dwa_planner_ros: public ControllerManager, public DynamicWindow::planner{
public:
    explicit dwa_planner_ros(StatePtr stateEstimator);


protected:
    void execute(const tf2::Transform& current_pose) override;
    void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);
    void rviz_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void obstacle_callback(geometry_msgs::msg::PoseArray::SharedPtr msg);

protected:
    void publish_short_horizon_traj(Traj& traj);
    void publish_obstacles();




private:
    ParamPtr2 parameters_;
    bool initialized_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr traj_pub_, obs_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obs_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_sub_;

    DynamicWindow::Config config_;
    Control control_;
    Obstacle obstacles_;
    tf2::Transform goal_pose_;

    std::once_flag obs_flag_;

    static tf2::Transform poseToTransform(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        tf2::Transform pose;
        double x, y, z;
        x = msg->pose.position.x;
        y = msg->pose.position.y;
        z = msg->pose.position.z;

        tf2::Quaternion q;
        q.setX(msg->pose.orientation.x);
        q.setY(msg->pose.orientation.y);
        q.setZ(msg->pose.orientation.z);
        q.setW(msg->pose.orientation.w);

        pose.setOrigin(tf2::Vector3(x, y, z));
        pose.setRotation(q);
        return pose;
    }
};


#endif //CREATE3_CONTROLLER_DWA_PLANNER_ROS_H
