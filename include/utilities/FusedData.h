//
// Created by redwan on 7/16/23.
//

#ifndef CREATE3_CONTROLLER_FUSEDDATA_H
#define CREATE3_CONTROLLER_FUSEDDATA_H
#include <vector>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/twist.hpp>

namespace model
{
    enum DATA_TYPE{
        ODOM = 0,
        APRILTAG,
        CMD_VEL,
        FUSION,
        SIM
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
}

#endif //CREATE3_CONTROLLER_FUSEDDATA_H
