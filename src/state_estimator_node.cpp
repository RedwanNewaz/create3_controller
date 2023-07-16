//
// Created by airlab on 3/8/23.
//
#include "../include/model.hpp"


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto stateEstimator = std::make_shared<model::JointStateEstimator> (options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(stateEstimator);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}