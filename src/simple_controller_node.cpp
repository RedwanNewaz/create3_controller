//
// Created by roboticslab on 4/17/23.
//

#include "../include/controller.hpp"



int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::executors::SingleThreadedExecutor executor;
    auto simpleController = std::make_shared<controller::UnicycleController>(options);
//    simpleController->overrideSafety(true);
    executor.add_node(simpleController);


    executor.spin();
    rclcpp::shutdown();

    return 0;
}
