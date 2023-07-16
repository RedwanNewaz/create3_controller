//
// Created by roboticslab on 11/22/22.
//

#ifndef CREATE3_CONTROLLER_DWA_PLANNER_ROS_H
#define CREATE3_CONTROLLER_DWA_PLANNER_ROS_H

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "dwa_planner.h"
#include "../manager.h"
#include "../../include/model.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include "utilities/param_manager2.h"


namespace controller
{

    class dwa_planner_ros: public manager, public DynamicWindow::planner{
    public:
        explicit dwa_planner_ros(const rclcpp::NodeOptions& options);


    protected:
        void execute(const tf2::Transform &current_pose) override;
        void set_goal_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void obstacle_callback(geometry_msgs::msg::PoseArray::SharedPtr msg);

    private:
        void publish_short_horizon_traj(Traj& traj);
        void publish_obstacles();
        tf2::Transform poseToTransform(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    private:
        ParamPtr2 parameters_;
        bool initialized_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr traj_pub_, obs_pub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obs_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_sub_;
        DynamicWindow::Config config_;
        Control control_;
        Obstacle obstacles_;
        tf2::Transform goal_pose_;
        std::once_flag obs_flag_;
    };

}


#endif //CREATE3_CONTROLLER_DWA_PLANNER_ROS_H
