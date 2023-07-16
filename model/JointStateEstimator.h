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
#include "filter/EKF.h"
#include "utilities/FusedData.h"
#include "tf2_ros/buffer_interface.h"
#include "LoggerCSV.h"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include <map>
//https://docs.ros.org/en/foxy/How-To-Guides/Overriding-QoS-Policies-For-Recording-And-Playback.html
//ros2 run tf2_ros static_transform_publisher 0 0 0  0 0 0 map odom

namespace model 
{

    class JointStateEstimator : public StateEstimatorBase{
    public:
        JointStateEstimator(const std::string &nodeName);
        ~JointStateEstimator()
        {
            delete odomInit_;
            delete apriltagInit_;
        }
        tf2::Transform getCurrentPose() override
        {
            return robotState_;
        }
        bool isInitialized() override
        {
            return (odomInit_ != nullptr && fusedData_->updateStatus[ODOM]);
        }



    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_, fusion_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
        rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr tag_sub_;

        const std::string fromFrameRel = "camera";
        std::string toFrameRel;
        std::string estimatorType_;
        tf2::Transform *odomInit_, *apriltagInit_;
        std::once_flag flagOdom_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_pub_, ekf_apriltag_pub_;
        std::unique_ptr<filter::EKF> ekf_;




        ///@brief variables related to tf listener
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<FusedData> fusedData_;
        tf2::TimePoint lastTFstamp_;

    private:
        LoggerCSV logger_;
        tf2::Transform robotState_;
        tf2::Vector3 lastDiff_;
        rclcpp::Time last_transform_;
        std::map<std::string, DATA_TYPE> sensorType_;
        int apriltagCounter_;


        std::unique_ptr<model::filter::ComplementaryFilter> lowpassFilter_;

        tf2::Transform get_state_from_odom();
        tf2::Transform get_state_from_apriltag();
        void get_state_from_fusion();
        double getYaw(const tf2::Quaternion& q);


    protected:
        void tf_to_odom(const tf2::Transform& t, nav_msgs::msg::Odometry& odom);
        void odom_to_tf(nav_msgs::msg::Odometry::SharedPtr odom, tf2::Transform& t);
        virtual void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);
        void cmd_callback(geometry_msgs::msg::Twist::SharedPtr msg);
        void lookupTransform() override;
        void sensorFusion() override;
    };
}


#endif //CREATE3_CONTROLLER_JOINTSTATEESTIMATOR_H
