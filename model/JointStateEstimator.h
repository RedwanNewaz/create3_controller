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
#include "tf2_ros/buffer_interface.h"
#include "LoggerCSV.h"
#include <map>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
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
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;


        std::string estimatorType_;
        tf2::Transform *odomInit_, *apriltagInit_;
        std::once_flag flagOdom_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_pub_, ekf_apriltag_pub_;
        std::unique_ptr<filter::EKF> ekf_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        enum DATA_TYPE{
            ODOM = 0,
            APRILTAG,
            CMD_VEL,
            FUSION
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

            void transformToVec(const tf2::Transform& trans, std::vector<double>&res)
            {
                auto origin = trans.getOrigin();
                auto q = trans.getRotation();

                double x = origin.x();
                double y = origin.y();
                tf2::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                double theta = fmod(yaw + M_PI, 2 * M_PI) - M_PI_2;
                res.push_back(x);
                res.push_back(y);
                res.push_back(theta);
            }
            void twistToVec(const geometry_msgs::msg::Twist& cmd, std::vector<double>&res)
            {
                res.push_back(cmd.linear.x);
                res.push_back(cmd.linear.y);
                res.push_back(cmd.angular.z);
            }


            std::vector<double> getLog()
            {
    //            {"cam_x", "cam_y", "cam_theta", "odom_x", "odom_y", "odom_theta", "cmd_vx", "cmd_vy", "cmd_wz", "odom_vx", "odom_vy", "odom_wz"};
                std::vector<double> result;
                transformToVec(apriltag, result);
                transformToVec(odom, result);
                twistToVec(cmd, result);
                twistToVec(odomTwsit, result);
                return result;

            }

        };

        ///@brief variables related to tf listener


        std::unique_ptr<FusedData> fusedData_;
        tf2::TimePoint lastTFstamp_;

    private:
        LoggerCSV logger_;
        tf2::Transform robotState_;
        std::map<std::string, DATA_TYPE> sensorType_;
        double odom_heading_offset_;

        std::unique_ptr<model::filter::ComplementaryFilter> lowpassFilter_;

        bool get_state_from_odom();
        bool get_state_from_apriltag();
        bool get_state_from_fusion();
        double getYaw(const tf2::Quaternion& q);


    protected:
        void tf_to_odom(const tf2::Transform& t, nav_msgs::msg::Odometry& odom);
        virtual void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);
        void cmd_callback(geometry_msgs::msg::Twist::SharedPtr msg);
        void lookupTransform(const Pose& pose) override;
        void sensorFusion() override;
    };
}


#endif //CREATE3_CONTROLLER_JOINTSTATEESTIMATOR_H
