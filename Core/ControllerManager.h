//
// Created by roboticslab on 6/2/23.
//

#ifndef CREATE3_CONTROLLER_CONTROLLERMANAGER_H
#define CREATE3_CONTROLLER_CONTROLLERMANAGER_H

#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "../state_estimator/StateViz.h"
#include "../state_estimator/JointStateEstimator.h"
#include "utilities/viz_objects.h"


#include <sensor_msgs/msg/joy.hpp>


class ControllerManager :public rclcpp::Node{
public:
    using StatePtr = std::shared_ptr<JointStateEstimator>;
    using TimePoint = std::chrono::system_clock::time_point;
    using Clock = std::chrono::system_clock;
    using Joy = sensor_msgs::msg::Joy;


    ControllerManager(const std::string &nodeName, const StatePtr &stateEstimator);

protected:
    void publish_cmd(double v, double w);
    void joy_callback(const Joy::SharedPtr msg);

    enum Mode{
        JOY_TELEOP,
        SAFE_LOCK,
        AUTO_NAV
    }controlMode_;
    virtual void execute(const tf2::Transform& current_pose) = 0;
protected:
    const double safetyBound_ = 0.75;
    const long lockTimeout_ = 15;
    StatePtr stateEstimator_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_;
    rclcpp::Subscription<Joy>::SharedPtr joy_;
    std::mutex mu_;
    TimePoint lock_time_;
private:
    void control_loop();
};


#endif //CREATE3_CONTROLLER_CONTROLLERMANAGER_H
