//
// Created by roboticslab on 6/2/23.
//

#ifndef CREATE3_CONTROLLER_manager_H
#define CREATE3_CONTROLLER_manager_H

#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "../include/model.hpp"


namespace controller
{

    class manager :public rclcpp::Node{
    public:
        using TimePoint = std::chrono::system_clock::time_point;
        using Clock = std::chrono::system_clock;


        manager(const rclcpp::NodeOptions& options);
        void overrideSafety(bool status);


    protected:
        void publish_cmd(double v, double w);
        void odom_to_tf(nav_msgs::msg::Odometry::SharedPtr msg, tf2::Transform &t);

        enum Mode{
            JOY_TELEOP,
            SAFE_LOCK,
            AUTO_NAV
        }controlMode_;
        virtual void execute(const tf2::Transform& current_pose) = 0;
    protected:
        const double safetyBound_ = 0.5;
        const long lockTimeout_ = 15;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fusion_sub_;
        rclcpp::Subscription<irobot_create_msgs::msg::IrIntensityVector>::SharedPtr intensity_sub_;
        std::vector<double> ir_values_;
        std::unique_ptr<model::filter::ComplementaryFilter> filter_;
        std::mutex mu_;
        TimePoint lock_time_;
    private:
        void control_loop();
        double maxIrValue();
        void intensity_callback(irobot_create_msgs::msg::IrIntensityVector::SharedPtr msg);
        bool safetyOverlook_;
        tf2::Transform robotState_;
        bool initialized_;
    protected:
        double safetyProb();
    };
}

#endif //CREATE3_CONTROLLER_manager_H
