//
// Created by redwan on 3/12/23.
//

#include "JointStateEstimator.h"
#include "tf2/time.h"
#include <time.h>

using namespace model;
Pose *Pose::nexigo_map = new model::Pose ({0.259, 1.737, 3.070, -0.014, 0.970, 0.226, 0.080}, "nexigo_cam->map");
Pose *Pose::logitec_map = new model::Pose ({-0.232, 1.258, 3.098, 0.996, -0.013, -0.026, 0.073}, "logitec_cam->map");

JointStateEstimator::JointStateEstimator(const std::string &nodeName) : StateEstimatorBase(nodeName)
{
    this->declare_parameter("sensor", "apriltag");
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

void JointStateEstimator::lookupTransform(const Pose& pose)
{


    auto transform = pose.getTransform();

    //FIX orientation
    auto q = transform.getRotation();
    double roll, pitch, yaw;
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    //TO see the actual detection
//    q.setRPY(roll, pitch + M_PI,  yaw + M_PI);
    q.setRPY(0, 0,  yaw);
    transform.setRotation(q);


    fusedData_->apriltag = transform;
    fusedData_->updateStatus[APRILTAG] = true;
    auto p = pose.getPoint();
    RCLCPP_INFO(get_logger(), "[apriltag] =  (%03lf, %03lf)", p.x, p.y);
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

    //check for suitable algorithm found
    if(sensorType_.find(estimatorType_) == sensorType_.end())
    {
        RCLCPP_ERROR(get_logger(), "No algorithm found for your input argument");
        return;
    }

    bool isUpdated;
    switch (sensorType_[estimatorType_]) {
        case APRILTAG: isUpdated = get_state_from_apriltag();break;
        case ODOM: isUpdated = get_state_from_odom(); break;
        case FUSION: isUpdated = get_state_from_fusion(); break;
    }

    if(!isUpdated)
        return;

    //skip orientation
    auto q = robotState_.getRotation();
    ekf_->update(robotState_, fusedData_->cmd, robotState_);
    robotState_.setRotation(q);

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

bool JointStateEstimator::get_state_from_odom() {

    if(odomInit_ == nullptr || !fusedData_->updateStatus[ODOM] || apriltagInit_ == nullptr)
        return false;

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


//    auto apriltag =  apriltagInit_->getOrigin();
//    auto odometry = odomInit_->getOrigin();
//    RCLCPP_INFO(get_logger(), "apriltag(x, y) = (%lf, %lf) ", apriltag.x(), apriltag.y());
//    RCLCPP_INFO(get_logger(), "odometry(x, y) = (%lf, %lf) ", odometry.x(), odometry.y());
    return true;
}

bool JointStateEstimator::get_state_from_apriltag() {
    if(fusedData_->updateStatus[APRILTAG])
    {
        robotState_ = fusedData_->apriltag;
        return true;
    }
    return false;
//    lowpassFilter_->update(fusedData_->apriltag, robotState_);
}

bool JointStateEstimator::get_state_from_fusion() {

    if(apriltagInit_ == nullptr || odomInit_ == nullptr)
        return false;

    //publish map to odom static transformation
//    auto M_t_O_init = *apriltagInit_;
//    auto O_t_O_init = *odomInit_;
//    auto M_t_O = M_t_O_init * O_t_O_init;
//    auto O_t_R = fusedData_->odom;
//    auto map_to_robot = M_t_O * O_t_R;
    get_state_from_odom();


    geometry_msgs::msg::TransformStamped t;
    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = std::string(get_namespace()) +  "/odom";

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    auto origin = robotState_.getOrigin();
    t.transform.translation.x = origin.x();
    t.transform.translation.y = origin.y();
    t.transform.translation.z = origin.z();

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    tf2::Quaternion q = robotState_.getRotation();
    auto yaw = getYaw(q);
    tf2::Quaternion viz;
    viz.setRPY(0, 0, yaw);
    t.transform.rotation.x = viz.x();
    t.transform.rotation.y = viz.y();
    t.transform.rotation.z = viz.z();
    t.transform.rotation.w = viz.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);



//# fuse odom + cmd_vel

//    get_state_from_odom();
//
//    ekf_->update(odomEkf, fusedData_->cmd, robotState_);

//# fuse cam + odom_vel

    get_state_from_apriltag();
    robotState_.setRotation(q);
    return true;
//    ekf_->update(robotState_, fusedData_->cmd, robotState_);
}

double JointStateEstimator::getYaw(const tf2::Quaternion &q) {
    double roll, pitch, yaw;
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    double theta = fmod(yaw + M_PI, 2 * M_PI);
    return theta;
}
