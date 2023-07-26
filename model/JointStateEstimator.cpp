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
    this->declare_parameter("odom_heading_offset", M_PI);
    // collect all the sensor and control information: (i) odom (ii) apriltag tf, and (iii) cmd_vel
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", qos, std::bind(
            &JointStateEstimator::odom_callback, this, std::placeholders::_1)
    );
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", qos   , std::bind(
            &JointStateEstimator::cmd_callback, this, std::placeholders::_1)
    );
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
    odom_heading_offset_ = this->get_parameter("odom_heading_offset").get_parameter_value().get<double>();

    RCLCPP_INFO(this->get_logger(), "%s State Estimator Initialized!!", estimatorType_.c_str());
    RCLCPP_INFO_STREAM(this->get_logger(), "[subscribed topic] odom " << odom_sub_->get_topic_name());
    RCLCPP_INFO_STREAM(this->get_logger(), "[subscribed topic] cmd_vel " << cmd_sub_->get_topic_name());

    // define sensor type
    sensorType_["odom"] = ODOM;
    sensorType_["apriltag"] = APRILTAG;
    sensorType_["fusion"] = FUSION;


    ekf_ = std::make_unique<filter::EKF>(1.0 / 200); // 200 Hz
    lowpassFilter_ = std::make_unique<filter::ComplementaryFilter>(0.85);

    // Initialize the transform broadcaster
    tf_broadcaster_ =  std::make_unique<tf2_ros::TransformBroadcaster>(*this);

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
        current.setOrigin(tf2::Vector3(t.transform.translation.x,
                                                    t.transform.translation.y,
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
        q.setRPY(0, 0, yaw + M_PI);

//        q.setRPY(roll, pitch + M_PI, 3 * M_PI_2 - yaw);

        current.setRotation(q);

        fusedData_->apriltag = current;
        fusedData_->updateStatus[APRILTAG] = true;

    } catch (const tf2::TransformException & ex) {
           RCLCPP_INFO(
                   this->get_logger(), "Could not transform %s to %s: %s",
                   toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
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
    if(odomInit_ == nullptr || !fusedData_->updateStatus[ODOM])
        return;

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
    nav_msgs::msg::Odometry msg;
    tf_to_odom(robotState_, msg);
    ekf_odom_pub_->publish(msg);

    // reset
    if(!std::count(fusedData_->updateStatus.begin(), fusedData_->updateStatus.end(), false))
    {
        fusedData_ = std::make_unique<FusedData>();
    }


//    logger_.addRow(fusedData_->getLog());

}

void JointStateEstimator::get_state_from_odom() {

    auto cameraToRobot = fusedData_->apriltag;
    auto odomToRobot = fusedData_->odom;

    auto origin = odomToRobot.getOrigin();
    origin = odomInit_->getOrigin() - origin;

    double x = origin.y();
    double y = -origin.x();
    origin.setX(x);
    origin.setY(y);

    origin = origin + apriltagInit_->getOrigin();


    auto q = odomToRobot.getRotation();
    auto yaw = getYaw(q);
    q.setRPY(0, 0, yaw);

    robotState_.setOrigin(origin);
    robotState_.setRotation(q);


    auto diffOdomToTag =  origin - cameraToRobot.getOrigin();
    auto apriltag =  apriltagInit_->getOrigin();
    auto odometry = odomInit_->getOrigin();
    RCLCPP_INFO(get_logger(), "diffOdomToTag(x, y) = (%lf, %lf) ", diffOdomToTag.x(), diffOdomToTag.y());
    RCLCPP_INFO(get_logger(), "apriltag(x, y) = (%lf, %lf) ", apriltag.x(), apriltag.y());
    RCLCPP_INFO(get_logger(), "odometry(x, y) = (%lf, %lf) ", odometry.x(), odometry.y());

}

void JointStateEstimator::get_state_from_apriltag() {

    lowpassFilter_->update(fusedData_->apriltag, robotState_);
}

void JointStateEstimator::get_state_from_fusion() {

    if(apriltagInit_ == nullptr)
        return;

    //publish map to odom static transformation
    auto M_t_O_init = *apriltagInit_;
    auto O_t_O_init = *odomInit_;
    auto M_t_O = M_t_O_init * O_t_O_init;
    auto O_t_R = fusedData_->odom;


    auto map_to_robot = M_t_O * O_t_R;

    geometry_msgs::msg::TransformStamped t;
    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = std::string(get_namespace()) +  "/odom";

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    auto origin = map_to_robot.getOrigin();
    t.transform.translation.x = origin.x();
    t.transform.translation.y = origin.y();
    t.transform.translation.z = origin.z();

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    tf2::Quaternion q = map_to_robot.getRotation();
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);



//# fuse odom + cmd_vel

//    get_state_from_odom();
//
//    ekf_->update(odomEkf, fusedData_->cmd, robotState_);

//# fuse cam + odom_vel

    get_state_from_apriltag();
//    ekf_->update(robotState_, fusedData_->cmd, robotState_);
}

double JointStateEstimator::getYaw(const tf2::Quaternion &q) {
    double roll, pitch, yaw;
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    double theta = fmod(yaw + M_PI, 2 * M_PI);
    return theta;
}
