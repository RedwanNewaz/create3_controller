//
// Created by redwan on 6/5/23.
//
#include "../include/controller.hpp"
#include "../include/view.hpp"
#include "../include/model.hpp"


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::string param =  (argc > 1) ? argv[1] : "";


    auto stateEstimator = std::make_shared<model::GazeboStateEstimator> ("gazeboStateEstimator");
    auto stateViz = std::make_shared<view::ControllerViz>("controllerViz", "odom");

    rclcpp::executors::MultiThreadedExecutor executor;
//    auto node1 = std::make_shared<controller::dwa_planner_ros>(stateEstimator);
    auto node1 = std::make_shared<controller::UnicycleController>("simpleController", stateEstimator);
    node1->overrideSafety(true);

    executor.add_node(node1);
    executor.add_node(stateEstimator);
    executor.add_node(stateViz);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
