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
        GazeboStateEstimator(const std::string &nodeName) : JointStateEstimator(nodeName),
                                                                                _isInitialized(false) {}

        bool isInitialized() override {
            return _isInitialized;
        }

        tf2::Transform getCurrentPose() override {
            return _current;
        }

    protected:
        void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg) override
        {
//            RCLCPP_INFO(get_logger(), "odom callback received");
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
        }

    private:
        bool _isInitialized;
        tf2::Transform _current;
    };


}
#endif //CREATE3_CONTROLLER_GAZEBOSTATEESTIMATOR_H
