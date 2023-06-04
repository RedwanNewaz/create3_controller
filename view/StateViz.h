//
// Created by redwan on 3/12/23.
//

#ifndef CREATE3_CONTROLLER_STATEVIZ_H
#define CREATE3_CONTROLLER_STATEVIZ_H
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>

using namespace std::chrono_literals;

namespace view
{
    class StateViz: public rclcpp::Node{
    public:
        explicit StateViz(const std::string& nodeName);
    private:
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr create3_state_pub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr create3_state_sub_, create3_state_sub2_, create3_state_sub3_;
        rclcpp::TimerBase::SharedPtr timer_;
        enum COLOR{
            RED,
            GREEN,
            DEFAULT
        };
        std::unordered_map<int, std::vector<geometry_msgs::msg::Point>> pubData_;
    protected:
        void state_callback(nav_msgs::msg::Odometry::SharedPtr msg, const COLOR& color);
        void publish_traj();
    };
}


#endif //CREATE3_CONTROLLER_STATEVIZ_H
