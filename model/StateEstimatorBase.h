//
// Created by redwan on 3/12/23.
//

#ifndef CREATE3_CONTROLLER_STATEESTIMATORBASE_H
#define CREATE3_CONTROLLER_STATEESTIMATORBASE_H
#include <rclcpp/rclcpp.hpp>
#include "filter/ComplementaryFilter.h"
#include <irobot_create_msgs/msg/ir_intensity_vector.hpp>
#include <chrono>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include "Pose.h"
const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

using namespace std::chrono_literals;

namespace model
{
    class StateMachine
    {
        int state_;
    public:
        StateMachine():state_(0)
        {

        }
        int update(bool logitec, bool nexigo)
        {

            if(state_ == 0 && nexigo && !logitec)
            {
                state_ = 2;
            }
            if(state_ == 0 && !nexigo && logitec)
            {
                state_ = 1;
            }

            if(state_ == 0 && nexigo && logitec)
            {
                state_ = 2; // priority goes to nexigo
            }

            if(state_ == 1 &&  nexigo && logitec)
            {
                state_ = 1;
            }

            if(state_ == 1 &&  !nexigo && logitec)
            {
                state_ = 1;
            }

            if(state_ == 1 && nexigo && !logitec)
            {
                state_ = 2; // nexigo
            }


            if(state_ == 2 && nexigo && !logitec)
            {
                state_ = 2;
            }
            if(state_ == 2 && !nexigo && logitec)
            {
                state_ = 1;
            }
            if(state_ == 2 && nexigo && logitec)
            {
                state_ = 2;
            }



            if (!logitec && !nexigo)
            {
                state_ = 0;
            }
            return state_;
        }

    };

    class StateEstimatorBase: public rclcpp::Node
    {
    public:
        StateEstimatorBase(const std::string &nodeName) : Node(nodeName)
        {
            timer_ = this->create_wall_timer(15ms, [this] { timer_callback(); });
            // prepare for transformation
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            intensity_sub_ = this->create_subscription<irobot_create_msgs::msg::IrIntensityVector>("ir_intensity", qos, std::bind(
                    &StateEstimatorBase::intensity_callback, this, std::placeholders::_1)
            );
            ir_values_.resize(7);
            // Call on_timer function every second
            filter_ = std::make_unique<filter::ComplementaryFilter>(0.99);
        }
        double safetyProb()
        {
            const double mu = 10.0;
            const double std = 2.0;
            double x = std::min(maxIrValue(), mu ) - 1e-6;
            const double normFactor = 1.0 / std * sqrt(2 * M_PI);
            double collisionProb = normFactor * exp(-0.5 * (mu - x) / std);
            return std::max(0.0, 1.0 - collisionProb);
        }

        virtual tf2::Transform getCurrentPose() = 0;
        virtual bool isInitialized() = 0;

    protected:
        virtual void lookupTransform(const Pose& pose) = 0;
        virtual void sensorFusion() = 0;

        void timer_callback()
        {
            geometry_msgs::msg::TransformStamped logitecCamPose, nexigoCamPose;
            auto logitecCam = getTransformation("logitec_cam", toFrameRel, logitecCamPose);
            auto nexigoCam = getTransformation("nexigo_cam", toFrameRel, nexigoCamPose);
            int state = sm_.update(logitecCam, nexigoCam);
            switch (state) {
                case 1:
                    lookupTransform(Pose(logitecCamPose));
                    break;
                case 2:
                    lookupTransform(Pose(nexigoCamPose));
                    break;
            }

            RCLCPP_INFO(get_logger(), "state = %d", state);



            sensorFusion();
        }
        double maxIrValue()
        {
            if(ir_values_.empty())
                return 0;
            return *std::max_element(ir_values_.begin(), ir_values_.end());
        }
        void intensity_callback(irobot_create_msgs::msg::IrIntensityVector::SharedPtr msg)
        {
            std::vector<double> mes(7);
            for (int i = 0; i < 7; ++i) {
                mes[i] = msg->readings[i].value;
            }
            filter_->update(mes, ir_values_);
        }
    private:
        rclcpp::Subscription<irobot_create_msgs::msg::IrIntensityVector>::SharedPtr intensity_sub_;
        std::vector<double> ir_values_;
        std::unique_ptr<filter::ComplementaryFilter> filter_;
    protected:
        rclcpp::TimerBase::SharedPtr timer_;
        std::string toFrameRel;
        bool getTransformation(const std::string& fromFrameRel, const std::string &toFrameRel, geometry_msgs::msg::TransformStamped& transformStamped)
        {
            bool success = false;
            try{
                transformStamped = tf_buffer_->lookupTransform(
                        fromFrameRel, toFrameRel,
                        tf2::TimePointZero);
                success = true;

            }
            catch (const tf2::TransformException & ex) {
                RCLCPP_INFO(
                        this->get_logger(), "Could not transform %s to %s: %s",
                        toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
            }

            return success;
        }

    private:
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        StateMachine sm_;
    };
}

#endif //CREATE3_CONTROLLER_STATEESTIMATORBASE_H
