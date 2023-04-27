//
// Created by roboticslab on 4/17/23.
//

#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "../state_estimator/StateViz.h"
#include "../state_estimator/JointStateEstimator.h"
#include "utilities/viz_objects.h"

typedef std::shared_ptr<JointStateEstimator> StatePtr;

inline tf2::Transform poseToTransform(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
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

class simple_controller_ros: public rclcpp::Node
{
public:
    simple_controller_ros(const std::string &nodeName, const StatePtr &stateEstimator) :
    Node(nodeName), stateEstimator_(stateEstimator)
    {
        initialized_ = false;
        cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",1);
        timer_ = this->create_wall_timer(30ms, std::bind(&simple_controller_ros::timer_callback, this));
        rviz_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 10, std::bind(
                &simple_controller_ros::rviz_callback, this, std::placeholders::_1)
        );

    }
protected:
    void timer_callback()
    {
        if(!initialized_ || !stateEstimator_->isInitialized())
            return;

        auto goal_position = goal_pose_.getOrigin();
        goal_position.setZ(0.0);


        tf2::Transform current_pose = stateEstimator_->getCurrentPose();
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
            vx = -vx;

        double cmd_v = kpRho * vx;
        double cmd_w = kpAlpha * alpha;

        if (error < goal_radius)
        {
            if(initialized_)
                RCLCPP_INFO(get_logger(), "[simple_controller] goal reached :-)");
            initialized_ = false;
            cmd_v = cmd_w = 0;
        }

        publish_cmd(cmd_v, cmd_w);


    }

    void rviz_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        const std::lock_guard<std::mutex> lk(mu_);
        goal_pose_ = poseToTransform(msg);
        initialized_ = true;
    }

    void publish_cmd(double v, double w) {
        const std::lock_guard<std::mutex> lk(mu_);
        geometry_msgs::msg::Twist cmd_vel;
        double safe = stateEstimator_->safetyProb();
        if(safe >= 0.75) // at least 50% safe
        {
            cmd_vel.linear.x = v;
            cmd_vel.angular.z = w;
//            RCLCPP_INFO(this->get_logger(), "[Collision Prob = %lf] sending cmd = (%lf, %lf)", 1 - safe, v, w);
        }
        else
        {
//            RCLCPP_WARN(this->get_logger(), "[Collision Prob = %lf] ignoring cmd = (%lf, %lf)", 1 - safe, v, w);
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
        }

        cmd_vel_->publish(cmd_vel);
    }


private:
    StatePtr stateEstimator_;
    bool initialized_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_sub_;
    tf2::Transform goal_pose_;
    std::mutex mu_;

};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::string param =  (argc > 1) ? argv[1] : "";
//    StatePtr stateEstimator = make_shared<state_estimator>(param);

    auto stateEstimator = std::make_shared<JointStateEstimator> ("stateEstimator");
    auto stateViz = std::make_shared<StateViz>("stateViz");
    auto controllerViz = std::make_shared<viz_objects>();

    rclcpp::executors::MultiThreadedExecutor executor;
    auto node1 = std::make_shared<simple_controller_ros>("simpleController", stateEstimator);
    executor.add_node(node1);
    executor.add_node(stateEstimator);
    executor.add_node(stateViz);
    executor.add_node(controllerViz);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
