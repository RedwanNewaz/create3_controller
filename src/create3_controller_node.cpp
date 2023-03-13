#include "create3_controller/dwa_planner_ros.h"
#include "utilities/viz_objects.h"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::string param =  (argc > 1) ? argv[1] : "";
    StatePtr stateEstimator = make_shared<state_estimator>(param);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto node1 = std::make_shared<dwa_planner_ros>(stateEstimator->get_ptr());
    auto node2 = std::make_shared<viz_objects>();
    executor.add_node(node1);
    executor.add_node(node2);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
