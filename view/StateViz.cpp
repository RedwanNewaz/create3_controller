//
// Created by redwan on 3/12/23.
//

#include "StateViz.h"
using namespace view;

StateViz::StateViz(const std::string& nodeName, const std::string& frameName):Node(nodeName), frameName_(frameName)
{
    create3_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("ekf/odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        state_callback(msg, DEFAULT);});
    create3_state_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/ekf/apriltag/viz", 10);
    timer_ = this->create_wall_timer(1s, [this] { publish_traj(); });
}

void StateViz::state_callback(nav_msgs::msg::Odometry::SharedPtr msg, const COLOR& color)
{
    // convert odom to viz marker
    visualization_msgs::msg::Marker marker;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.header.stamp  = get_clock()->now();
    marker.header.frame_id  = frameName_;
//    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://irobot_create_description/meshes/body_visual.dae";
    marker.id = static_cast<int>(color);
    marker.ns = get_namespace();
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0; // arrow scale 0.2 roomba scale 1.0
    switch (color) {
        case RED: marker.color.r = 1; break;
        case GREEN: marker.color.g = 1; break;
        default:
            marker.color.r = marker.color.g  = marker.color.b = 0.66;

    }
    marker.color.a = 0.85;
    marker.pose = msg->pose.pose;

    create3_state_pub_->publish(marker);
    // show traj
    pubData_[color].push_back(marker.pose.position);

}

void StateViz::publish_traj()
{
    auto color = DEFAULT;
    if(pubData_[color].empty())
        return;

    visualization_msgs::msg::Marker trajMarker;
    std::copy(pubData_[color].begin(), pubData_[color].end(), std::back_inserter(trajMarker.points));
    trajMarker.id = 202;
    trajMarker.action = visualization_msgs::msg::Marker::ADD;
    trajMarker.header.stamp = get_clock()->now();
    trajMarker.header.frame_id = "map";

    trajMarker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    trajMarker.ns = get_namespace();
    trajMarker.color.g = 1;
    trajMarker.color.a = 0.7;
    trajMarker.scale.x = trajMarker.scale.y = trajMarker.scale.z = 0.08;
    create3_state_pub_->publish(trajMarker);
}