//
// Created by roboticslab on 6/2/23.
//

#ifndef CREATE3_CONTROLLER_SIMPLECONTROLLER_H
#define CREATE3_CONTROLLER_SIMPLECONTROLLER_H
#include "../Core/ControllerManager.h"
#include "../Core/DockHelper.h"

class SimpleController: public ControllerManager {

public:
    SimpleController(const std::string &nodeName, const StatePtr &stateEstimator);
    static tf2::Transform poseToTransform(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

private:
    void rviz_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    bool initialized_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_sub_;
    tf2::Transform goal_pose_;
protected:
    void execute(const tf2::Transform &current_pose) override;


};


#endif //CREATE3_CONTROLLER_SIMPLECONTROLLER_H
