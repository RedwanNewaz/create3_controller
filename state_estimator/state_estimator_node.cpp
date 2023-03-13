//
// Created by airlab on 3/8/23.
//
#include "JointStateEstimator.h"
#include "StateViz.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto stateEstimator = std::make_shared<JointStateEstimator> ("stateEstimator");
    auto stateViz = std::make_shared<StateViz>("stateViz");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(stateEstimator);
    executor.add_node(stateViz);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}