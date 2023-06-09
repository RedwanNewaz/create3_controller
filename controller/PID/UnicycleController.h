//
// Created by roboticslab on 6/2/23.
//

#ifndef CREATE3_CONTROLLER_UNICYCLECONTROLLER_H
#define CREATE3_CONTROLLER_UNICYCLECONTROLLER_H
#include "../manager.h"

namespace controller
{
    class UnicycleController: public manager {

    public:
        UnicycleController(const std::string &nodeName, const StatePtr &stateEstimator);
        static tf2::Transform poseToTransform(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    private:
        void set_goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        bool initialized_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
        tf2::Transform goal_pose_;
    protected:
        void execute(const tf2::Transform &current_pose) override;
    };

}

#endif //CREATE3_CONTROLLER_UNICYCLECONTROLLER_H
