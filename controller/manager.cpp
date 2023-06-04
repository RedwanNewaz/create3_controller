//
// Created by roboticslab on 6/2/23.
//

#include "manager.h"
using namespace controller;

manager::manager(const std::string &nodeName, const manager::StatePtr &stateEstimator):
Node(nodeName), stateEstimator_(stateEstimator)
{
    cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",1);
    timer_ = this->create_wall_timer(30ms, std::bind(&manager::control_loop, this));
    controlMode_ = JOY_TELEOP;
}

void manager::publish_cmd(double v, double w) {
    const std::lock_guard<std::mutex> lk(mu_);
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = v;
    cmd_vel.angular.z = w;
    cmd_vel_->publish(cmd_vel);
}

void manager::control_loop() {
    if(!stateEstimator_->isInitialized())
        return;


    double safe = stateEstimator_->safetyProb();

    if(controlMode_ == AUTO_NAV)
    {
        if(safe >= safetyBound_) // at least 75% safe
        {
            tf2::Transform current_pose = stateEstimator_->getCurrentPose();
            execute(current_pose);
        }
        else
        {
            controlMode_ = SAFE_LOCK;
            publish_cmd(0, 0);
            lock_time_ = Clock::now();
        }
    }

    else if (controlMode_ == SAFE_LOCK)
    {


        publish_cmd(0, 0);
        Clock::duration elaspsedTime = Clock::now() - lock_time_;
        auto waitTime = std::chrono::duration_cast<std::chrono::seconds>(elaspsedTime).count();
        RCLCPP_INFO(get_logger(), "[safety ]: prob = %lf | please wait %ld s", safe, lockTimeout_ - waitTime);
        if( waitTime >= lockTimeout_)
        {
            controlMode_ = JOY_TELEOP;
        }
    }

}
