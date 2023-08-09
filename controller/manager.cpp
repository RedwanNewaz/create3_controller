//
// Created by roboticslab on 6/2/23.
//

#include "manager.h"
using namespace controller;

manager::manager(const std::string& nodeName, const rclcpp::NodeOptions& options):
Node(nodeName)
{
    this->declare_parameter("safetyOverlook", true);
    cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",1);
    timer_ = this->create_wall_timer(30ms, std::bind(&manager::control_loop, this));
    controlMode_ = JOY_TELEOP;
    safetyOverlook_ =  this->get_parameter("safetyOverlook").get_parameter_value().get<bool>();

    // sensor fusion happens using dual EKF with robot localization package
    // check out the config/dual_ekf.yaml file for the topic and configuraiton
    initialized_ = false;
    fusion_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("ekf/fusion", qos, [&]
            (nav_msgs::msg::Odometry::SharedPtr msg){
        odom_to_tf(msg, robotState_);
        initialized_ = true;
    });
    // introduce safety
    intensity_sub_ = this->create_subscription<irobot_create_msgs::msg::IrIntensityVector>("ir_intensity", qos, std::bind(
            &manager::intensity_callback, this, std::placeholders::_1)
    );
    ir_values_.resize(7);
    // Call on_timer function every second
    filter_ = std::make_unique<model::filter::ComplementaryFilter>(0.99);


}

void manager::publish_cmd(double v, double w) {
    const std::lock_guard<std::mutex> lk(mu_);
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = v;
    cmd_vel.angular.z = w;
    cmd_vel_->publish(cmd_vel);
}

void manager::control_loop() {
    if(!initialized_)
        return;


    double safe = safetyProb();

    if(controlMode_ == AUTO_NAV)
    {
        if(safe >= safetyBound_ || safetyOverlook_) // at least 75% safe
        {
            tf2::Transform current_pose = robotState_;
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

void manager::overrideSafety(bool status) {
    safetyOverlook_ = status;

}


void manager::odom_to_tf(nav_msgs::msg::Odometry::SharedPtr msg, tf2::Transform &t) {


    t.setOrigin(tf2::Vector3(msg->pose.pose.position.x,
                             msg->pose.pose.position.y,
                             msg->pose.pose.position.z
    ));


    t.setRotation(tf2::Quaternion(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
    ));
}


double manager::safetyProb()
{
    const double mu = 10.0;
    const double std = 2.0;
    double x = std::min(maxIrValue(), mu ) - 1e-6;
    const double normFactor = 1.0 / std * sqrt(2 * M_PI);
    double collisionProb = normFactor * exp(-0.5 * (mu - x) / std);
    return std::max(0.0, 1.0 - collisionProb);
}

double manager::maxIrValue()
{
    if(ir_values_.empty())
        return 0;
    return *std::max_element(ir_values_.begin(), ir_values_.end());
}
void manager::intensity_callback(irobot_create_msgs::msg::IrIntensityVector::SharedPtr msg)
{
    std::vector<double> mes(7);
    for (int i = 0; i < 7; ++i) {
        mes[i] = msg->readings[i].value;
    }
    filter_->update(mes, ir_values_);
}
