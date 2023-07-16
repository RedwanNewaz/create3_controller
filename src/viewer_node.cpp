//
// Created by redwan on 7/16/23.
//
#include "../include/view.hpp"



int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto controllerViz = std::make_shared<view::ControllerViz>("controllerViz");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(controllerViz);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
