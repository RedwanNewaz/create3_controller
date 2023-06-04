//
// Created by airlab on 3/8/23.
//
#include "../include/model.hpp"
#include "../include/view.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto stateEstimator = std::make_shared<model::JointStateEstimator> ("stateEstimator");
    auto stateViz = std::make_shared<view::StateViz>("stateViz");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(stateEstimator);
    executor.add_node(stateViz);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}