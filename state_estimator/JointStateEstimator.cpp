//
// Created by redwan on 3/12/23.
//

#include "JointStateEstimator.h"

JointStateEstimator::JointStateEstimator(const std::string &nodeName) : StateEstimatorBase(nodeName)
{
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
    RCLCPP_INFO(this->get_logger(), "Joint State Estimator Initialized!!");
    ekf_ = std::make_unique<EKF>(1.0 / 200); // 20 Hz
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
    geometry_msgs::msg::TransformStamped t;
    try {
        t = tf_buffer_->lookupTransform(
                fromFrameRel, toFrameRel,
                tf2::TimePointZero);
        fusedData_->apriltag.setOrigin(tf2::Vector3(-t.transform.translation.y,
                                                    -t.transform.translation.x,
                                                    t.transform.translation.z
        ));
        fusedData_->apriltag.setRotation(
                tf2::Quaternion(
                        t.transform.rotation.x,
                        t.transform.rotation.y,
                        t.transform.rotation.z,
                        t.transform.rotation.w)
        );



        fusedData_->updateStatus[APRILTAG] = true;
//            RCLCPP_INFO(this->get_logger(), "Found transform %s to %s",  toFrameRel.c_str(), fromFrameRel.c_str());
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


//    auto q = fusedData_->odom.getRotation();
//    fusedData_->apriltag.setRotation(q);
//    tf2::Transform OdomFilter(fusedData_->apriltag);
    tf2::Transform OdomFilter(fusedData_->apriltag);

    double roll, pitch, yaw;
    auto q = fusedData_->odom.getRotation();
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    double theta = fmod(yaw + M_PI, 2 * M_PI) - M_PI_2;
    q.setRPY(0, 0, theta);
    OdomFilter.setRotation(q);



    ekf_->update(OdomFilter, fusedData_->odomTwsit, OdomFilter);
//     consider control input
    ekf_->update(OdomFilter, fusedData_->cmd, OdomFilter);
    OdomFilter.setRotation(q);

//    auto newAngle = OdomFilter.getRotation();
//    double finalAngle = fmod(newAngle.getAngle() + 3 * M_PI_2, 2 * M_PI);
//        finalAngle = std::min(2 * M_PI - finalAngle, finalAngle);
//    newAngle.setRPY(0, 0, finalAngle);
//    OdomFilter.setRotation(newAngle);



    // convert to odom message
    nav_msgs::msg::Odometry msg1;
    tf_to_odom(OdomFilter, msg1);
    ekf_odom_pub_->publish(msg1);

    // reset
    if(!std::count(fusedData_->updateStatus.begin(), fusedData_->updateStatus.end(), false))
    {
        fusedData_ = std::make_unique<FusedData>();
    }
}