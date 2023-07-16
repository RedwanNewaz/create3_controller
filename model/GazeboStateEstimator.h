//
// Created by redwan on 6/4/23.
//

#ifndef CREATE3_CONTROLLER_GAZEBOSTATEESTIMATOR_H
#define CREATE3_CONTROLLER_GAZEBOSTATEESTIMATOR_H
#include "JointStateEstimator.h"

namespace model
{
    class GazeboStateEstimator: public JointStateEstimator
    {
    public:
        GazeboStateEstimator(const rclcpp::NodeOptions& options) : JointStateEstimator(options),
                                                                                _isInitialized(false), topicName_("odom") {
            timer_->cancel();
        }

        GazeboStateEstimator(const rclcpp::NodeOptions& options, const std::string& topicName) : JointStateEstimator(options),
                                                            _isInitialized(false), topicName_(topicName) {
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(topicName, qos, std::bind(
                    &GazeboStateEstimator::odom_callback, this, std::placeholders::_1)
            );
            std::string pubTopic = topicName + "/filtered";
            odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(pubTopic, 10);
            timer_->cancel();



        }

        bool isInitialized() override {
            return _isInitialized;
        }

        tf2::Transform getCurrentPose() override {
            return _current;
        }

    protected:
        void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg) override
        {
//            RCLCPP_INFO_STREAM(get_logger(), topicName_);
            auto getYaw = [](const tf2::Quaternion& q)
            {
                double roll, pitch, yaw;
                tf2::Matrix3x3 m(q);
                m.getRPY(roll, pitch, yaw);
                double theta = fmod(yaw + M_PI, 2 * M_PI) - M_PI_2;
                return theta;
            };

            _current.setOrigin(tf2::Vector3(msg->pose.pose.position.x,
                                                    msg->pose.pose.position.y,
                                                    msg->pose.pose.position.z
            ));

            tf2::Quaternion q(msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y,
                            msg->pose.pose.orientation.z,
                            msg->pose.pose.orientation.w);

            double yaw = getYaw(q);
            q.setRPY(0, 0, yaw);

            _current.setRotation(q);
            _isInitialized = true;

//            nav_msgs::msg::Odometry odom;
//            tf_to_odom(_current, odom);
//            odom_pub_->publish(odom);

        }

    private:
        bool _isInitialized;
        std::string topicName_;
        tf2::Transform _current;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    };


}
#ifdef GAZEBO_SIM
    RCLCPP_COMPONENTS_REGISTER_NODE(model::GazeboStateEstimator)
#endif

#endif //CREATE3_CONTROLLER_GAZEBOSTATEESTIMATOR_H
