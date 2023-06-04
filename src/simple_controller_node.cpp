//
// Created by roboticslab on 4/17/23.
//

#include "../include/controller.hpp"
#include "../include/view.hpp"



int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::string param =  (argc > 1) ? argv[1] : "";

    auto stateEstimator = std::make_shared<model::JointStateEstimator> ("stateEstimator");
    auto controllerViz = std::make_shared<view::ControllerViz>("controllerViz");

    rclcpp::executors::MultiThreadedExecutor executor;
    auto simpleController = std::make_shared<controller::UnicycleController>("simpleController", stateEstimator);
    executor.add_node(simpleController);
    executor.add_node(stateEstimator);
    executor.add_node(controllerViz);


    executor.spin();
    rclcpp::shutdown();

    return 0;
}
