//
// Created by airlab on 7/28/23.
//

#ifndef CREATE3_CONTROLLER_POSE_H
#define CREATE3_CONTROLLER_POSE_H


#include <string>
#include <map>
#include <unordered_map>
#include "tf2/transform_datatypes.h"
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

inline void printTransformation(const tf2::Transform& transform)
{
    std::cout << "Translation: (" << transform.getOrigin().x()
              << ", " << transform.getOrigin().y()
              << ", " << transform.getOrigin().z() << ")" << std::endl;
    std::cout << "Rotation: (" << transform.getRotation().x()
              << ", " << transform.getRotation().y()
              << ", " << transform.getRotation().z()
              << ", " << transform.getRotation().w() << ")" << std::endl;
}

namespace model
{
    class Pose{
    public:

        explicit Pose(const geometry_msgs::msg::TransformStamped& transform)
        {
            auto p = transform.transform.translation;
            auto q = transform.transform.rotation;
            position_ = {p.x, p.y, p.z};
            orientation_ = {q.x, q.y, q.z, q.w};
            timestamp_ = transform.header.stamp.sec;

            if(transform.header.frame_id == "logitec_cam")
            {
                auto L_t_R = getTransform();
                auto M_t_R = mapToLogitecTag(L_t_R);
                update(M_t_R);
                frame_ = "map->/" + transform.header.frame_id;
            }
            else if(transform.header.frame_id == "nexigo_cam")
            {
                auto N_t_R = getTransform();
                auto M_t_R = mapToNexigoTag(N_t_R);
                update(M_t_R);
                frame_ = "map->/" + transform.header.frame_id;
            }
            else
                frame_ = transform.header.frame_id + "->" + transform.child_frame_id;
        }

        double_t  getTimestamp() const
        {
            return timestamp_;
        }

        explicit Pose(const std::array<double, 7>& static_tf, const std::string frame):frame_(frame)
        {
            for (int i = 0; i < 3; ++i) {
                position_[i] = static_tf[i];
            }

            for (int j = 3; j < static_tf.size(); ++j) {
                orientation_[j] = static_tf[j];
            }
        }

        explicit Pose(const tf2::Transform& tf, const std::string frame):frame_(frame)
        {

            auto p = tf.getOrigin();
            position_ = {p.x(), p.y(), p.z()};

            auto q = tf.getRotation();
            orientation_ = {q.x(), q.y(), q.z(), q.w()};
        }

        [[nodiscard]] tf2::Transform getTransform() const
        {
            tf2::Transform transform;
            transform.setOrigin(tf2::Vector3(position_[0], position_[1], position_[2]));
            transform.setRotation(tf2::Quaternion(orientation_[0], orientation_[1], orientation_[2], orientation_[3]));
            return transform;
        }
        [[nodiscard]] std::tuple<double, double, double> getState() const
        {
            double x, y, roll, pitch, yaw;

            x = position_[0];
            y = position_[1];

            tf2::Quaternion q(orientation_[0], orientation_[1], orientation_[2], orientation_[3]);
            tf2::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);


            return std::make_tuple(x, y, yaw);
        }

        [[nodiscard]] std::string getFrameID() const
        {
            return frame_;
        }

        [[nodiscard]] std::string toString() const
        {
            double x, y, theta;
            std::tie(x, y, theta) = getState();
            return std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(theta);
        }

        geometry_msgs::msg::Point getPoint() const
        {
            geometry_msgs::msg::Point p;
            p.x = position_[0];
            p.y = position_[1];
            p.z = position_[2];
            return p;
        }


    private:
        std::array<double, 3> position_;
        std::array<double, 4> orientation_;
        std::string frame_;
        double_t timestamp_;
    public:
        static Pose *nexigo_map;
        static Pose *logitec_map;

    private:
        static tf2::Transform mapToLogitecTag(const tf2::Transform& L_t_R)
        {
            auto L_t_M = logitec_map->getTransform();
            auto M_t_R = L_t_M.inverseTimes(L_t_R);

            auto origin = M_t_R.getOrigin();
            double y = origin.getY();
            origin.setY(-y);
            M_t_R.setOrigin(origin);

            // fix rotation
            auto q = M_t_R.getRotation();
            double roll, pitch, yaw;
            tf2::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);
            q.setRPY(0, 0,     M_PI - yaw  );
            M_t_R.setRotation(q);

            return M_t_R;
        }

        static tf2::Transform mapToNexigoTag(const tf2::Transform& N_t_R)
        {
            auto N_t_M = nexigo_map->getTransform();
            auto M_t_R = N_t_M.inverseTimes(N_t_R);

            auto origin = M_t_R.getOrigin();
            double x = origin.getX();
            origin.setX(-x);
            M_t_R.setOrigin(origin);


            auto q = M_t_R.getRotation();
            double roll, pitch, yaw;
            tf2::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);
            q.setRPY(0, 0,     2 * M_PI - yaw  );
            M_t_R.setRotation(q);

            return M_t_R;
        }

        void update(const tf2::Transform& tf)
        {
            auto p = tf.getOrigin();
            auto q = tf.getRotation();
            position_ = {p.x(), p.y(), p.z()};
            orientation_ = {q.x(), q.y(), q.z(), q.w()};
        }
    };
}



#endif //CREATE3_CONTROLLER_POSE_H
