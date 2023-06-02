//
// Created by roboticslab on 4/17/23.
//

#include "SimpleController.h"



int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::string param =  (argc > 1) ? argv[1] : "";

    auto stateEstimator = std::make_shared<JointStateEstimator> ("stateEstimator");
    auto stateViz = std::make_shared<StateViz>("stateViz");
    auto controllerViz = std::make_shared<viz_objects>();

//    auto undockRobot = std::make_shared<UndockClientNode>("undock");
//    auto dockRobot = std::make_shared<DockServoClientNode>("dock_servo");

    rclcpp::executors::MultiThreadedExecutor executor;
    auto node1 = std::make_shared<SimpleController>("simpleController", stateEstimator);
    executor.add_node(node1);
    executor.add_node(stateEstimator);
    executor.add_node(stateViz);
    executor.add_node(controllerViz);

//    executor.add_node(undockRobot);
//    executor.add_node(dockRobot);

    executor.spin();
    rclcpp::shutdown();

    return 0;
}
