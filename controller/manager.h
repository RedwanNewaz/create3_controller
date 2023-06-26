//
// Created by roboticslab on 6/2/23.
//

#ifndef CREATE3_CONTROLLER_manager_H
#define CREATE3_CONTROLLER_manager_H

#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "../include/model.hpp"

namespace controller
{

    class manager :public rclcpp::Node{
    public:
        using StatePtr = std::shared_ptr<model::JointStateEstimator>;
        using TimePoint = std::chrono::system_clock::time_point;
        using Clock = std::chrono::system_clock;


        manager(const std::string &nodeName, const StatePtr &stateEstimator);
        void overrideSafety(bool status);

    protected:
        void publish_cmd(double v, double w);

        enum Mode{
            JOY_TELEOP,
            SAFE_LOCK,
            AUTO_NAV
        }controlMode_;
        virtual void execute(const tf2::Transform& current_pose) = 0;
    protected:
        const double safetyBound_ = 0.5;
        const long lockTimeout_ = 15;
        StatePtr stateEstimator_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_;
        std::mutex mu_;
        TimePoint lock_time_;
    private:
        void control_loop();
        bool safetyOverlook_;
    };
}

#endif //CREATE3_CONTROLLER_manager_H
