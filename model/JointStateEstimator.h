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
#include <rclcpp_components/register_node_macro.hpp>

//https://docs.ros.org/en/foxy/How-To-Guides/Overriding-QoS-Policies-For-Recording-And-Playback.html
//ros2 run tf2_ros static_transform_publisher 0 0 0  0 0 0 map odom


#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
    #define StateEstimator_CPP_EXPORT __attribute__ ((dllexport))
    #define StateEstimator_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define StateEstimator_CPP_EXPORT __declspec(dllexport)
    #define StateEstimator_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef StateEstimator_CPP_BUILDING_DLL
    #define StateEstimator_CPP_PUBLIC StateEstimator_CPP_EXPORT
  #else
    #define StateEstimator_CPP_PUBLIC StateEstimator_CPP_IMPORT
  #endif
  #define StateEstimator_CPP_PUBLIC_TYPE StateEstimator_CPP_PUBLIC
  #define StateEstimator_CPP_LOCAL
#else
#define StateEstimator_CPP_EXPORT __attribute__ ((visibility("default")))
#define StateEstimator_CPP_IMPORT
#if __GNUC__ >= 4
#define StateEstimator_CPP_PUBLIC __attribute__ ((visibility("default")))
#define StateEstimator_CPP_LOCAL  __attribute__ ((visibility("hidden")))
#else
#define StateEstimator_CPP_PUBLIC
    #define StateEstimator_CPP_LOCAL
#endif
#define StateEstimator_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif


namespace model 
{
    
    class JointStateEstimator : public StateEstimatorBase{
    public:
        StateEstimator_CPP_PUBLIC
        explicit JointStateEstimator(const rclcpp::NodeOptions& options);
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
