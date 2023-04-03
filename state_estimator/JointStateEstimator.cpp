//
// Created by redwan on 3/12/23.
//

#include "JointStateEstimator.h"
#include "tf2/time.h"
#include <time.h>

JointStateEstimator::JointStateEstimator(const std::string &nodeName) : StateEstimatorBase(nodeName)
{
    this->declare_parameter("sensor", "odom");
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


    RCLCPP_INFO(this->get_logger(), "%s State Estimator Initialized!!", estimatorType_.c_str());
    ekf_ = std::make_unique<EKF>(1.0 / 200); // 200 Hz

    // enable logger
    std::vector<std::string> header = {"cam_x", "cam_y", "cam_theta", "odom_x", "odom_y", "odom_theta", "cmd_vx", "cmd_vy", "cmd_wz", "odom_vx", "odom_vy", "odom_wz"};
    const std::string outputFolder = "/home/roboticslab/colcon_ws/src/create3_controller/results";
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
    fusedData_->odom.setRotation(tf2::Quaternion(msg->pose.pose.orientation.x,
                                                 msg->pose.pose.orientation.y,
                                                 msg->pose.pose.orientation.z,
                                                 msg->pose.pose.orientation.w
    ));
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
//        t = tf_buffer_->lookupTransform(                fromFrameRel, toFrameRel,
//                get_clock()->now(), rclcpp::Duration::from_seconds(0.1));
        current.setOrigin(tf2::Vector3(-t.transform.translation.y,
                                                    -t.transform.translation.x,
                                                    t.transform.translation.z
        ));
        current.setRotation(
                tf2::Quaternion(
                        t.transform.rotation.x,
                        t.transform.rotation.y,
                        t.transform.rotation.z,
                        t.transform.rotation.w)
        );

        fusedData_->apriltag = current;
        fusedData_->updateStatus[APRILTAG] = true;

    } catch (const tf2::TransformException & ex) {
//            RCLCPP_INFO(
//                    this->get_logger(), "Could not transform %s to %s: %s",
//                    toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
//            return;
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



    tf2::Transform OdomFilter;

    if(estimatorType_ == "fusion")
    {

        auto getYaw = [](const tf2::Quaternion& q)
        {
            double roll, pitch, yaw;
            tf2::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);
            double theta = fmod(yaw + M_PI, 2 * M_PI) - M_PI_2;
            return theta;
        };

        double cam_init_theta = getYaw(apriltagInit_->getRotation());
        double odom_init_theta = getYaw(odomInit_->getRotation());

//        double thetaInit = (cam_init_theta + odom_init_theta) / 2;
        //TODO make thetaInit as a parameter
        double thetaInit = M_PI / 12.0;
        double x = fusedData_->odom.getOrigin().x();
        double y = fusedData_->odom.getOrigin().y();

        double x0 = x * cos(thetaInit) - y * sin(thetaInit);
        double y0 = x * sin(thetaInit) + y * cos(thetaInit);

        double cam_x0 = apriltagInit_->getOrigin().x();
        double cam_y0 = apriltagInit_->getOrigin().y();

        double odom_x0 = odomInit_->getOrigin().x();
        double odom_y0 = odomInit_->getOrigin().y();

        x = x0 + cam_x0 - odom_x0;
        y = y0 + cam_y0 - odom_y0;




//# fuse odom + cmd_vel

        tf2::Transform odomEkf;
        odomEkf.setOrigin(tf2::Vector3(x, y, 0));
        auto q = fusedData_->odom.getRotation();
        double theta = getYaw(q);
        q.setRPY(0, 0, theta);
        odomEkf.setRotation(q);
        ekf_->update(odomEkf, fusedData_->cmd, OdomFilter);

//# fuse cam + odom_vel

        tf2::Transform camEkf;
        double cam_x = fusedData_->apriltag.getOrigin().x();
        double cam_y = fusedData_->apriltag.getOrigin().y();
        camEkf.setOrigin(tf2::Vector3(cam_x, cam_y, 0));
        // use odom orientation for yaw
        camEkf.setRotation(q);
        ekf_->update(OdomFilter, fusedData_->odomTwsit, OdomFilter);


    }
    else if(estimatorType_ == "odom")
    {
        OdomFilter = fusedData_->odom;
        double roll, pitch, yaw;
        auto q = fusedData_->odom.getRotation();
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        double theta = fmod(yaw + M_PI, 2 * M_PI) - M_PI_2;
        q.setRPY(0, 0, theta);
        OdomFilter.setRotation(q);
    }
    else if(estimatorType_ == "apriltag")
    {
        OdomFilter = fusedData_->apriltag;
    }

    // convert to odom message
    nav_msgs::msg::Odometry msg;
    tf_to_odom(OdomFilter, msg);
    ekf_odom_pub_->publish(msg);

    // reset
//    if(!std::count(fusedData_->updateStatus.begin(), fusedData_->updateStatus.end(), false))
//    {
//        fusedData_ = std::make_unique<FusedData>();
//    }
//

//    logger_.addRow(fusedData_->getLog());

}