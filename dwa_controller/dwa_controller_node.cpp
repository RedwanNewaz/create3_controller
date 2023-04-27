#include "create3_controller/dwa_planner_ros.h"
#include "utilities/viz_objects.h"
#include "../state_estimator/StateViz.h"


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::string param =  (argc > 1) ? argv[1] : "";
//    StatePtr stateEstimator = make_shared<state_estimator>(param);

    auto stateEstimator = std::make_shared<JointStateEstimator> ("stateEstimator");
    auto stateViz = std::make_shared<StateViz>("stateViz");

    rclcpp::executors::MultiThreadedExecutor executor;
    auto node1 = std::make_shared<dwa_planner_ros>(stateEstimator);
    auto node2 = std::make_shared<viz_objects>();
    executor.add_node(node1);
    executor.add_node(node2);
    executor.add_node(stateEstimator);
    executor.add_node(stateViz);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
