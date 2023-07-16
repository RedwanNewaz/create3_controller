//
// Created by redwan on 3/12/23.
//

#include "JointStateEstimator.h"
#include "tf2/time.h"
#include <time.h>

using namespace model; 

JointStateEstimator::JointStateEstimator(const std::string &nodeName) : StateEstimatorBase(nodeName)
{
    this->declare_parameter("sensor", "odom");
    this->declare_parameter("robotTag", "tag36h11:7");
    this->declare_parameter("logOutput", "/var/tmp");
    // collect all the sensor and control information: (i) odom (ii) apriltag tf, and (iii) cmd_vel
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", qos, std::bind(
            &JointStateEstimator::odom_callback, this, std::placeholders::_1)
    );
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", qos   , std::bind(
            &JointStateEstimator::cmd_callback, this, std::placeholders::_1)
    );

    apriltagCounter_ = 0;
    tag_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>("/detections", qos, [&]
            (apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg){
        apriltagCounter_++;
        RCLCPP_INFO_STREAM(get_logger(), "apriltagCounter_ " << apriltagCounter_);
        fusedData_->updateStatus[APRILTAG] = apriltagCounter_ >= 10;
    });

    // prepare for transformation
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // save them as tf::transform format
    fusedData_ = std::make_unique<FusedData>();
    // ekf odom
    ekf_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("ekf/odom", 10);
    ekf_apriltag_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("ekf/apriltag", 10);


    estimatorType_ = this->get_parameter("sensor").get_parameter_value().get<std::string>();
    toFrameRel = this->get_parameter("robotTag").get_parameter_value().get<std::string>();

    RCLCPP_INFO(this->get_logger(), "%s State Estimator Initialized!!", estimatorType_.c_str());
    RCLCPP_INFO_STREAM(this->get_logger(), "[subscribed topic] odom " << odom_sub_->get_topic_name());
    RCLCPP_INFO_STREAM(this->get_logger(), "[subscribed topic] cmd_vel " << cmd_sub_->get_topic_name());

    // define sensor type
    sensorType_["odom"] = ODOM;
    sensorType_["apriltag"] = APRILTAG;
    sensorType_["fusion"] = FUSION;


    ekf_ = std::make_unique<filter::EKF>(1.0 / 200); // 200 Hz
    lowpassFilter_ = std::make_unique<filter::ComplementaryFilter>(0.99);



    // enable logger
    std::vector<std::string> header = {"cam_x", "cam_y", "cam_theta", "odom_x", "odom_y", "odom_theta", "cmd_vx", "cmd_vy", "cmd_wz", "odom_vx", "odom_vy", "odom_wz"};
    const std::string outputFolder = this->get_parameter("logOutput").get_parameter_value().get<std::string>();;
    logger_.addHeader(header);
    logger_.setOutputFolder(outputFolder);

}

void JointStateEstimator::tf_to_odom(const tf2::Transform& t, nav_msgs::msg::Odometry& odom)
{
    odom.header.frame_id = "odom";
    auto pos = t.getOrigin();
    auto ori = t.getRotation();
    odom.header.stamp = get_clock()->now();

    odom.pose.pose.position.x = pos.x();
    odom.pose.pose.position.y = pos.y();
    odom.pose.pose.position.z = pos.z();

    odom.pose.pose.orientation.x = ori.x();
    odom.pose.pose.orientation.y = ori.y();
    odom.pose.pose.orientation.z = ori.z();
    odom.pose.pose.orientation.w = ori.w();

    odom.twist.twist = fusedData_->cmd;
}

void JointStateEstimator::odom_callback(nav_msgs::msg::Odometry::SharedPtr msg)
{
    fusedData_->odom.setOrigin(tf2::Vector3(msg->pose.pose.position.x,
                                            msg->pose.pose.position.y,
                                            msg->pose.pose.position.z
    ));


    tf2::Quaternion q(msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w
    );

    double yaw = getYaw(q);
    q.setRPY(0, 0, yaw);

    fusedData_->odom.setRotation(q);
    fusedData_->odomTwsit = msg->twist.twist;
    fusedData_->updateStatus[ODOM] = true;
}

void JointStateEstimator::lookupTransform()
{

    tf2::Transform current;
//    tf2::Time timeout = rclcpp::Duration::from_seconds(0.1);
    try {
        auto t = tf_buffer_->lookupTransform(
                fromFrameRel, toFrameRel,
                tf2::TimePointZero);
//        auto t = tf_buffer_->lookupTransform(                fromFrameRel, toFrameRel,
//                get_clock()->now(), rclcpp::Duration::from_seconds(0.2));

        current.setOrigin(tf2::Vector3(-t.transform.translation.y,
                                                    -t.transform.translation.x,
                                                    t.transform.translation.z
        ));

        tf2::Quaternion tagq(
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3 m(tagq);
        m.getRPY(roll, pitch, yaw);

        tf2::Quaternion q;
        q.setRPY(roll, pitch + M_PI, 3 * M_PI_2 - yaw);
        current.setRotation(q);
        lowpassFilter_->update(current, fusedData_->apriltag );


    } catch (const tf2::TransformException & ex) {
//           RCLCPP_INFO(
//                   this->get_logger(), "Could not transform %s to %s: %s",
//                   toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
           return;
    }

    // generate consensus using sensor fusion algorithm
    // check if all sensors are updated or not: there is no false in the array
    if(!std::count(fusedData_->updateStatus.begin(), fusedData_->updateStatus.end(), false))
    {
        // TODO should this update need to be done frequently or once?

        std::call_once(flagOdom_, [&](){
            odomInit_ = new tf2::Transform(fusedData_->odom);
            apriltagInit_ = new tf2::Transform(fusedData_->apriltag);
        });

    }
}
void JointStateEstimator::cmd_callback(geometry_msgs::msg::Twist::SharedPtr msg)
{
    fusedData_->cmd = *msg.get();
    fusedData_->updateStatus[CMD_VEL] = true;
}

void JointStateEstimator::sensorFusion()
{
    if(odomInit_ == nullptr || apriltagInit_ == nullptr )
    {
//        RCLCPP_ERROR(get_logger(), "Sensors are not initialized yet");
        return;
    }

    //check for suitable algorithm found
    if(sensorType_.find(estimatorType_) == sensorType_.end())
    {
        RCLCPP_ERROR(get_logger(), "No algorithm found for your input argument");
        return;
    }

    switch (sensorType_[estimatorType_]) {
        case APRILTAG: get_state_from_apriltag();break;
        case ODOM: get_state_from_odom(); break;
        case FUSION: get_state_from_fusion(); break;
    }

    // convert to odom message
//    nav_msgs::msg::Odometry msg;
//    tf_to_odom(robotState_, msg);
//    ekf_odom_pub_->publish(msg);

    // reset
    fusedData_ = std::make_unique<FusedData>();

//    logger_.addRow(fusedData_->getLog());

}

void JointStateEstimator::get_state_from_odom() {

    if (!fusedData_->updateStatus[ODOM])
        return;

    auto cameraToRobot = fusedData_->apriltag;

    auto odomToRobot = fusedData_->odom;
    auto cameraToOdom = apriltagInit_->inverseTimes(*odomInit_);
    auto cameraToRobot_odom = cameraToOdom.inverseTimes(odomToRobot);

    auto q = cameraToRobot_odom.getRotation();
    auto origin = cameraToRobot_odom.getOrigin() - cameraToOdom.getOrigin() - odomInit_->getOrigin();


    double x = origin.y();
    double y = -origin.x();
    origin.setX(x);
    origin.setY(y);


    auto diffOdomToTag =  origin - cameraToRobot.getOrigin();
    auto apriltag =  apriltagInit_->getOrigin();
    auto odometry = odomInit_->getOrigin();
    RCLCPP_INFO(get_logger(), "diffOdomToTag(x, y) = (%lf, %lf) ", diffOdomToTag.x(), diffOdomToTag.y());
    RCLCPP_INFO(get_logger(), "apriltag(x, y) = (%lf, %lf) ", apriltag.x(), apriltag.y());
    RCLCPP_INFO(get_logger(), "odometry(x, y) = (%lf, %lf) ", odometry.x(), odometry.y());

    if(!fusedData_->updateStatus[APRILTAG])
        lastDiff_ = diffOdomToTag;
    origin -= lastDiff_;

    robotState_.setOrigin(origin);
    robotState_.setRotation(odomToRobot.getRotation());

    nav_msgs::msg::Odometry odom;
    tf_to_odom(robotState_, odom);
    odom.header.stamp = get_clock()->now();
    odom.header.frame_id = "map";
    ekf_odom_pub_->publish(odom);


}

void JointStateEstimator::get_state_from_apriltag() {

    if(!fusedData_->updateStatus[APRILTAG])
        return;

    robotState_ = fusedData_->apriltag;

//    lowpassFilter_->update(fusedData_->apriltag, robotState_);
//    auto diffApriltag = robotState_.getOrigin() - fusedData_->apriltag.getOrigin();
//    RCLCPP_INFO(get_logger(), "driftApriltag(x, y) = (%lf, %lf) ", diffApriltag.x(), diffApriltag.y());

    nav_msgs::msg::Odometry apriltag;
    tf_to_odom(robotState_, apriltag);
    apriltag.header.stamp = get_clock()->now();
    apriltag.header.frame_id = "map";
    ekf_apriltag_pub_->publish(apriltag);

}

void JointStateEstimator::get_state_from_fusion() {

    bool initUpdate = fusedData_->updateStatus[APRILTAG] && fusedData_->updateStatus[ODOM];
    auto setTransform = [](tf2::Transform *target, const tf2::Transform t)
    {
        target->setOrigin(t.getOrigin());
        target->setRotation(t.getRotation());
    };

//# fuse apriltag + cmd_vel
    if (fusedData_->updateStatus[APRILTAG])
    {
        get_state_from_apriltag();
        if(initUpdate)
            setTransform(apriltagInit_, robotState_);



    }
//# fuse odom + cmd_vel
    else if (fusedData_->updateStatus[ODOM])
    {
        get_state_from_odom();

        if(initUpdate)
            setTransform(odomInit_, robotState_);

    }

}

double JointStateEstimator::getYaw(const tf2::Quaternion &q) {
    double roll, pitch, yaw;
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    double theta = fmod(yaw + M_PI, 2 * M_PI) - M_PI_2;
    return theta;
}
