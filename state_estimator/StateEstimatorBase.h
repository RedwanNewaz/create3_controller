//
// Created by redwan on 3/12/23.
//

#ifndef CREATE3_CONTROLLER_STATEESTIMATORBASE_H
#define CREATE3_CONTROLLER_STATEESTIMATORBASE_H
#include <rclcpp/rclcpp.hpp>
#include "ComplementaryFilter.h"
#include <irobot_create_msgs/msg/ir_intensity_vector.hpp>
#include <chrono>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>

const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

using namespace std::chrono_literals;

class StateEstimatorBase: public rclcpp::Node
{
public:
    StateEstimatorBase(const std::string &nodeName) : Node(nodeName)
    {
        intensity_sub_ = this->create_subscription<irobot_create_msgs::msg::IrIntensityVector>("ir_intensity", qos, std::bind(
                &StateEstimatorBase::intensity_callback, this, std::placeholders::_1)
        );
        ir_values_.resize(7);
        // Call on_timer function every second
        filter_ = std::make_unique<ComplementaryFilter>(0.99);
        timer_ = this->create_wall_timer(15ms, [this] { timer_callback(); });

    }

    double safetyProb()
    {
        const double mu = 10.0;
        const double std = 2.0;
        double x = std::min(maxIrValue(), mu ) - 1e-6;
        const double normFactor = 1.0 / std * sqrt(2 * M_PI);
        double collisionProb = normFactor * exp(-0.5 * (mu - x) / std);
        return 1.0 - collisionProb;
    }

protected:
    virtual void lookupTransform() = 0;
    virtual void sensorFusion() = 0;

    void timer_callback()
    {
        lookupTransform();
        sensorFusion();
        double safe = safetyProb();
        if(safe <= 0.5)
            RCLCPP_INFO(get_logger(), "[safety ]: prob = %lf", safe);
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
    std::unique_ptr<ComplementaryFilter> filter_;
    rclcpp::TimerBase::SharedPtr timer_;


};

#endif //CREATE3_CONTROLLER_STATEESTIMATORBASE_H
