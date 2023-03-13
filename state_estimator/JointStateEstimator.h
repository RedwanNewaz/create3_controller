//
// Created by redwan on 3/12/23.
//

#ifndef CREATE3_CONTROLLER_JOINTSTATEESTIMATOR_H
#define CREATE3_CONTROLLER_JOINTSTATEESTIMATOR_H
#include "StateEstimatorBase.h"
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <nav_msgs/msg/odometry.hpp>
#include "EKF.h"

//https://docs.ros.org/en/foxy/How-To-Guides/Overriding-QoS-Policies-For-Recording-And-Playback.html
//ros2 run tf2_ros static_transform_publisher 0 0 0  0 0 0 map odom

class JointStateEstimator : public StateEstimatorBase{
public:
    JointStateEstimator(const std::string &nodeName);
    ~JointStateEstimator()
    {
        delete odomInit_;
        delete apriltagInit_;
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

    const std::string fromFrameRel = "camera";
    const std::string toFrameRel = "tag36h11:7";
    tf2::Transform *odomInit_, *apriltagInit_;
    std::once_flag flagOdom_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_pub_, ekf_apriltag_pub_;
    std::unique_ptr<EKF> ekf_;
    enum DATA_TYPE{
        ODOM = 0,
        APRILTAG,
        CMD_VEL
    };

    struct FusedData{
        FusedData()
        {
            updateStatus[ODOM] = updateStatus[APRILTAG] = updateStatus[CMD_VEL] = false;
        }
        tf2::Transform odom;
        tf2::Transform apriltag;
        geometry_msgs::msg::Twist cmd, odomTwsit;
        std::array<bool, 3> updateStatus;

    };

    ///@brief variables related to tf listener
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<FusedData> fusedData_;

protected:
    void tf_to_odom(const tf2::Transform& t, nav_msgs::msg::Odometry& odom);
    void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);
    void cmd_callback(geometry_msgs::msg::Twist::SharedPtr msg);
    void lookupTransform() override;
    void sensorFusion() override;
};


#endif //CREATE3_CONTROLLER_JOINTSTATEESTIMATOR_H
