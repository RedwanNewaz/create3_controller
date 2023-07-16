//
// Created by roboticslab on 6/2/23.
//

#include "UnicycleController.h"
namespace controller
{
    UnicycleController::UnicycleController(const rclcpp::NodeOptions& options)
            : manager(options) {
        initialized_ = false;
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 10, std::bind(
                &UnicycleController::set_goal_callback, this, std::placeholders::_1)
        );

    }

    UnicycleController::~UnicycleController()
    {

    }

    void UnicycleController::set_goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        const std::lock_guard<std::mutex> lk(mu_);
        goal_pose_ = poseToTransform(msg);
        initialized_ = true;
        controlMode_ = AUTO_NAV;
    }

    tf2::Transform UnicycleController::poseToTransform(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        tf2::Transform pose;
        double x, y, z;
        x = msg->pose.position.x;
        y = msg->pose.position.y;
        z = msg->pose.position.z;

        tf2::Quaternion q;
        q.setX(msg->pose.orientation.x);
        q.setY(msg->pose.orientation.y);
        q.setZ(msg->pose.orientation.z);
        q.setW(msg->pose.orientation.w);

        pose.setOrigin(tf2::Vector3(x, y, z));
        pose.setRotation(q);
        return pose;
    }

    void UnicycleController::execute(const tf2::Transform &current_pose) {
        if(!initialized_)
            return;

        auto goal_position = goal_pose_.getOrigin();
        goal_position.setZ(0.0);



        auto curr_position = current_pose.getOrigin();
        curr_position.setZ(0.0);

        tf2::Vector3 position_diff = goal_position - curr_position;

        double error = tf2::tf2Distance(curr_position, goal_position);

        auto getYaw = [](const tf2::Quaternion& q)
        {
            double roll, pitch, yaw;
            tf2::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);
            return yaw;
        };

        double theta = getYaw(current_pose.getRotation()) - M_PI_2;
        double alpha = atan2(position_diff.y(), position_diff.x());
//        alpha = std::min(2 * M_PI - alpha, alpha);
        alpha = alpha  - theta  + M_PI;
        alpha = fmod(alpha, 2 * M_PI) - M_PI;

        const double kpRho = 0.50; // m/s
        const double kpAlpha = 0.95; // rad/s 0.35
        const double goal_radius = 0.3; // m


        double vx = std::min(1.0, error);
        if (alpha > M_PI_2 || alpha < - M_PI_2)
        {
            vx = -vx * 0;
        }

        double cmd_v = kpRho * vx;
        double cmd_w = kpAlpha * alpha;

        if (error < goal_radius)
        {
            if(initialized_)
                RCLCPP_INFO(get_logger(), "[simple_controller] goal reached :-)");
            initialized_ = false;
            // handover the controller to the joystick
            controlMode_ = JOY_TELEOP;
            cmd_v = cmd_w = 0;
        }

        publish_cmd(cmd_v, cmd_w);

    }

}



RCLCPP_COMPONENTS_REGISTER_NODE(controller::UnicycleController)