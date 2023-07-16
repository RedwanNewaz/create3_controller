#include "../include/controller.hpp"
#include "../include/view.hpp"
#include "../include/model.hpp"


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::string param =  (argc > 1) ? argv[1] : "";
//    StatePtr stateEstimator = make_shared<state_estimator>(param);


    auto stateViz = std::make_shared<view::ControllerViz>("controllerViz");
    rclcpp::NodeOptions options;
    rclcpp::executors::MultiThreadedExecutor executor;
    auto controller_node = std::make_shared<controller::dwa_planner_ros>(options);

    executor.add_node(controller_node);

    executor.add_node(stateViz);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
