//
// Created by roboticslab on 11/22/22.
//

#include "create3_controller/dwa_planner_ros.h"

dwa_planner_ros::dwa_planner_ros(StatePtr stateEstimator):Node("DWA"), stateEstimator_(stateEstimator) {
    initialized_ = false;

    // create subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(
            &dwa_planner_ros::odom_callback, this, std::placeholders::_1)
    );
    obs_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/obstacles", 10, [&](const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {return obstacle_callback(msg);});
    rviz_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(
            &dwa_planner_ros::rviz_callback, this, std::placeholders::_1)
    );

    // create publishers
    cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",1);
    traj_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("short_traj", 10);
    obs_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("obstacles", 10);
    timer_ = this->create_wall_timer(30ms, std::bind(&dwa_planner_ros::timer_callback, this));

    // initialize internal parameters
    control_[0] = control_[1] = 0;
    std::function<double(const string&)> f = [&](const string& param)
    {return stateEstimator_->parameters->get_param<double>(param);};
    config_.update_param(f);
    stateEstimator_->parameters->get_obstacles(obstacles_);
}


//-------------------------------MAIN LOOP--------------------------------------------------------

void dwa_planner_ros::timer_callback() {

    // don't execute until a goal location is given
    if(!initialized_)
        return;

    // compute state difference
    auto goal_position = goal_pose_.getOrigin();
    tf2::Transform current_pose = stateEstimator_->getCurrentPose();
    auto curr_position = current_pose.getOrigin();
    tf2::Vector3 position_diff = goal_position - curr_position;
    double current_angle = tf2::getYaw(current_pose.getRotation());

    // compute terminal condition
    double remainDist = sqrt(pow(position_diff.x(), 2) + pow(position_diff.y(), 2) );
    Point goal;
    goal[0] = goal_position.x();
    goal[1] = goal_position.y();

    // don't move if you reach to a goal region
    if (remainDist < config_.goal_radius)
    {
        initialized_ = false;
        control_[0] = control_[1] = 0;
    }
    else
    {
        // compute local trajectory using dynamic window
        State x({{curr_position.x(), curr_position.y(), current_angle, control_[0], control_[1]}});
        Traj ltraj = DynamicWindow::planner::compute_control(x, control_, config_, goal, obstacles_);
        publish_short_horizon_traj(ltraj);
    }

    // publish cmd_vel and update state
    publish_cmd(control_[0], control_[1]);
    stateEstimator_->add_cmd_vel(control_[0], control_[1]);

}


//-------------------------------ROS SUBSCRIBERS--------------------------------------------------------

void dwa_planner_ros::odom_callback(nav_msgs::msg::Odometry::SharedPtr msg) {
    const std::lock_guard<mutex> lk(mu_);
    stateEstimator_->odom_callback(msg);
    std::call_once(obs_flag_, [&](){ publish_obstacles(); });
}

void dwa_planner_ros::rviz_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    const std::lock_guard<mutex> lk(mu_);
    goal_pose_ = state_estimator::poseToTransform(msg);
    initialized_ = true;
    publish_obstacles();
}


void dwa_planner_ros::obstacle_callback(geometry_msgs::msg::PoseArray::SharedPtr msg) {
    const std::lock_guard<mutex> lk(mu_);
    obstacles_.clear();
    for(auto& pose: msg->poses)
    {
        obstacles_.push_back({pose.position.x, pose.position.y});
    }
}

//-------------------------------ROS PUBLISHERS--------------------------------------------------------

void dwa_planner_ros::publish_cmd(double v, double w) {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = v;
    cmd_vel.angular.z = w;
    cmd_vel_->publish(cmd_vel);
    //    RCLCPP_INFO(this->get_logger(), "%lf, %lf", v, w);
}

void dwa_planner_ros::publish_short_horizon_traj(Traj &traj) {
    geometry_msgs::msg::PoseArray msg;
    msg.header.stamp = this->get_clock()->now();
    for(auto & p: traj)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = p[0];
        pose.position.y = p[1];
        pose.position.z = 0;
        msg.poses.push_back(pose);
    }
    traj_pub_->publish(msg);
}

void dwa_planner_ros::publish_obstacles() {
    geometry_msgs::msg::PoseArray msg;
    msg.header.stamp = this->get_clock()->now();

    for(auto &item:obstacles_)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = item[0];
        pose.position.y = item[1];
        pose.position.z = 0;
        msg.poses.push_back(pose);
    }
    obs_pub_->publish(msg);
}

