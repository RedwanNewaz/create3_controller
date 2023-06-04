//
// Created by redwan on 6/4/23.
//

#ifndef CREATE3_CONTROLLER_CONTROLLERVIZ_H
#define CREATE3_CONTROLLER_CONTROLLERVIZ_H
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "StateViz.h"

namespace view {

    class ControllerViz: public StateViz{

    public:
        using MARKER = visualization_msgs::msg::Marker;
        using PATH_VEC = std::vector<std::vector<double>>;

        explicit ControllerViz(const std::string &nodeName);
        void short_horizon_traj_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
        void obstacle_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
        void rviz_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        MARKER gen_traj(const PATH_VEC &path);

    private:
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr traj_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obs_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_sub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;


    };




} // view

#endif //CREATE3_CONTROLLER_CONTROLLERVIZ_H
