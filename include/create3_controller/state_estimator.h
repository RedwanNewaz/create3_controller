//
// Created by redwan on 11/10/22.
//

#ifndef CREATE3_CONTROLLER_STATE_ESTIMATOR_H
#define CREATE3_CONTROLLER_STATE_ESTIMATOR_H
#include "COM.h"
#include "utilities/param_manager2.h"
#define STATE_DIM 5
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>


using namespace std;
class state_estimator;
typedef shared_ptr<state_estimator> StatePtr;

class state_estimator: public enable_shared_from_this<state_estimator>
{
public:
    state_estimator(const string& param_file)
    {
        state_.resize(STATE_DIM);
        for (int i = 0; i < STATE_DIM; ++i) {
            state_[i] = 0;
        }
        if(!param_file.empty())
            parameters = make_shared<param_manager2>(param_file);
    }
    void add_cmd_vel(double v, double w)
    {
        state_[3] = v;
        state_[4] = w;
//        control_queues_.push_back(make_pair(v, w));
    }

    std::pair<double, double> get_cmds()
    {
        auto val = control_queues_.back();
        control_queues_.pop_back();
        return val;
    }

    bool is_cmd_arrived()
    {
        return !control_queues_.empty();
    }

    static tf2::Transform odomToTransform(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        tf2::Transform pose;
        double x, y, z;
        x = msg->pose.pose.position.x;
        y = msg->pose.pose.position.y;
        z = msg->pose.pose.position.z;

        tf2::Quaternion q;
        q.setX(msg->pose.pose.orientation.x);
        q.setY(msg->pose.pose.orientation.y);
        q.setZ(msg->pose.pose.orientation.z);
        q.setW(msg->pose.pose.orientation.w);

        pose.setOrigin(tf2::Vector3(x, y, z));
        pose.setRotation(q);
        return pose;
    }

    static tf2::Transform poseToTransform(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        tf2::Transform pose;
        double x, y, z;
        x = msg->pose.position.x;
        y = msg->pose.position.y;
        z = msg->pose.position.z;

        tf2::Quaternion q;
        q.setX(msg->pose.orientation.x);
        q.setY(msg->pose.orientation.y);
        q.setZ(msg->pose.orientation.z);
        q.setW(msg->pose.orientation.w);

        pose.setOrigin(tf2::Vector3(x, y, z));
        pose.setRotation(q);
        return pose;
    }

    tf2::Transform getCurrentPose()
    {
        return current_pose_;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        state_[0] = msg->pose.pose.position.x;
        state_[1] = msg->pose.pose.position.y;

        tf2::Quaternion q;
        q.setX(msg->pose.pose.orientation.x);
        q.setY(msg->pose.pose.orientation.y);
        q.setZ(msg->pose.pose.orientation.z);
        q.setW(msg->pose.pose.orientation.w);


        state_[2] = tf2::getYaw(q);
        state_[3] = msg->twist.twist.linear.x;
        state_[4] = msg->twist.twist.angular.z;

//        DEBUG("[state_estimator]: updated " << state_[0] << " " << state_[1] << " " << state_[2] );

        // set current pose
        current_pose_.setOrigin(tf2::Vector3(state_[0], state_[1], state_[2]));
        current_pose_.setRotation(q);

    }
    StatePtr get_ptr()
    {
        return shared_from_this();
    }

    float at(int index)
    {
        return state_.at(index);
    }



    ParamPtr2 parameters;
private:
    std::vector<double>state_;
    std::deque<std::pair<double, double>> control_queues_;
    tf2::Transform current_pose_;
};

#endif //CREATE3_CONTROLLER_STATE_ESTIMATOR_H
