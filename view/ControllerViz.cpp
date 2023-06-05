//
// Created by redwan on 6/4/23.
//

#include "ControllerViz.h"

namespace view {
    ControllerViz::ControllerViz(const std::string &nodeName, const std::string &frameName) : StateViz(nodeName, frameName) {
        traj_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("short_traj", 10, [&](const geometry_msgs::msg::PoseArray::SharedPtr msg)
        {return short_horizon_traj_callback(msg);});
        rviz_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 10, std::bind(
                &ControllerViz::rviz_callback, this, std::placeholders::_1)
        );
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("dwa_create3", 10);
        obs_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("obstacles", 10, [&](const geometry_msgs::msg::PoseArray::SharedPtr msg)
        {return obstacle_callback(msg);});
    }
    void ControllerViz::obstacle_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        MARKER marker;
        marker.header.frame_id = frameName_;

        marker.header.stamp = this->get_clock()->now();
        marker.ns = "obstacles";
        marker.id = 101;
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;

        marker.scale.x = 0.95;
        marker.scale.y = 0.95;
        marker.scale.z = 0.95;

        marker.color.a = 0.95;
        marker.color.r = 1.0 ;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
//        RCLCPP_INFO(this->get_logger(), "reading obstacles_ positions");
        for(auto& pose: msg->poses)
        {
            geometry_msgs::msg::Point p;
            p.x = pose.position.x;
            p.y = pose.position.y;
            p.z = 0;
            RCLCPP_INFO(this->get_logger(), "[obstacles viz]: x = %lf | y = %lf ", p.x, p.y);
            marker.points.push_back(p);
        }
        marker_pub_->publish(marker);
    }

    void ControllerViz::short_horizon_traj_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        PATH_VEC path;
//        RCLCPP_INFO(get_logger(), "received short horizon traj");
        for(auto& pose: msg->poses)
        {
            path.push_back({pose.position.x, pose.position.y, pose.position.z});
        }
        auto marker = gen_traj(path);
        marker_pub_->publish(marker);
    }

    void ControllerViz::rviz_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        MARKER marker;
        marker.header.frame_id = frameName_;

        marker.header.stamp = this->get_clock()->now();
        marker.ns = "create3";
        marker.id = 2;
        marker.type = visualization_msgs::msg::Marker::SPHERE;

        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        marker.color.a = 0.75;
        marker.color.r = 1.0 ;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker.pose = msg->pose;
        marker_pub_->publish(marker);
    }


    ControllerViz::MARKER ControllerViz::gen_traj(const ControllerViz::PATH_VEC &path) {
        MARKER marker;
        marker.header.frame_id = frameName_;

        marker.header.stamp = this->get_clock()->now();
        marker.ns = "create3";
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;

        marker.scale.x = 0.02;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;

        marker.color.a = 0.75;
        marker.color.r = 0.0 ;
        marker.color.g = 1.0;
        marker.color.b = 1.0;

        for (auto& p:path)
        {
            geometry_msgs::msg::Point xx;
            xx.x = p[0];
            xx.y = p[1];
            xx.z = 0;
            marker.points.push_back(xx);
        }

        return marker;
    }
} // view