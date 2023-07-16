//
// Created by redwan on 3/11/23.
//

#include "EKF.h"
#include<iostream>
#include<random>
#include<cmath>

using namespace model::filter; 

// x_{t+1} = F@x_{t}+B@u_t
Eigen::Vector4f EKF::motion_model(const Eigen::Vector4f &x, const Eigen::Vector2f &u) const
{
    Eigen::Matrix4f F_;
    F_<<1.0,   0,   0,   0,
            0, 1.0,   0,   0,
            0,   0, 1.0,   0,
            0,   0,   0, 1.0;

    Eigen::Matrix<float, 4, 2> B_;
    B_<< DT * std::cos(x(2,0)),  0,
            DT * std::sin(x(2,0)),  0,
            0.0,  DT,
            1.0,  0.0;
    Eigen::Vector4f xPred =  F_ * x + B_ * u;

    return xPred;
}

Eigen::Matrix4f EKF::jacobF(const Eigen::Vector4f& x, const Eigen::Vector2f& u) const 
{
    Eigen::Matrix4f jF_ = Eigen::Matrix4f::Identity();
    float yaw = x(2);
    float v = u(0);
    jF_(0,2) = -DT * v * std::sin(yaw);
    jF_(0,3) = DT * std::cos(yaw);
    jF_(1,2) = DT * v * std::cos(yaw);
    jF_(1,3) = DT * std::sin(yaw);
    return jF_;
}

//observation mode H
Eigen::Vector3f EKF::observation_model(const Eigen::Vector4f& x) const
{
    Eigen::Matrix<float, 3, 4> H_;
    H_<< 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0;
    return H_ * x;
}

Eigen::Matrix<float, 3, 4> EKF::jacobH()const
{
    Eigen::Matrix<float, 3, 4> jH_;
    jH_<< 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0;
    return jH_;
}

void EKF::ekf_estimation(const Eigen::Vector3f& z, const Eigen::Vector2f& u)
{
    Eigen::Vector4f xPred = motion_model(xEst, u);
    Eigen::Matrix4f jF = jacobF(xPred, u);
    Eigen::Matrix4f PPred = jF * PEst * jF.transpose() + Q;

    Eigen::Matrix<float, 3, 4> jH = jacobH();
    Eigen::Vector3f zPred = observation_model(xPred);
    Eigen::Vector3f y = z - zPred;

//    // borrowed from https://github.com/JunshengFu/tracking-with-Extended-Kalman-Filter/blob/master/src/kalman_filter.cpp
//    // normalize the angle between -pi to pi
//    while(y(2) > M_PI_2){
//        y(2) -= M_PI_4;
//    }
//
//    while(y(2) < -M_PI_2){
//        y(2) += M_PI_4;
//    }

    Eigen::Matrix3f S = jH * PPred * jH.transpose() + R;
    Eigen::Matrix<float, 4, 3> K = PPred * jH.transpose() * S.inverse();
    xEst = xPred + K * y;
    PEst = (Eigen::Matrix4f::Identity() - K * jH) * PPred;
}

void EKF::update(const tf2::Transform &obs, const geometry_msgs::msg::Twist& cmd, tf2::Transform &res) {
    auto origin = obs.getOrigin();
    double roll, pitch, theta;
    auto q = obs.getRotation();
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, theta);

    double w = cmd.angular.z;
//    double v = sqrt(cmd.linear.x * cmd.linear.x + cmd.linear.y * cmd.linear.y );
    double v = cmd.linear.x ;

    if(!initialized)
    {
        xEst(0, 0) = origin.x();
        xEst(1, 0) = origin.y();
        xEst(2, 0) = theta;
        xEst(3, 0) = v;
        initialized = true;
    }

    Eigen::Vector3f z(origin.x(), origin.y(), theta);

    const Eigen::Vector2f u(v, w);
    ekf_estimation(z, u);

    res.setOrigin(tf2::Vector3(xEst(0, 0), xEst(1, 0), origin.z()));
    tf2::Quaternion q2;
    q2.setRPY(0, 0, xEst(2, 0));
    res.setRotation(q2);

}

EKF::EKF(const double dt) : DT(dt)
{
    initialized = false;

    PEst = Eigen::Matrix4f::Identity() * 0.1;

//    // Motional model covariance
    Q = Eigen::Matrix4f::Identity();
    Q(0,0)=0.1 * 0.05;
    Q(1,1)=0.1 * 0.05;
    Q(2,2)= (1.0/180 * M_PI);
//    Q(2,2)= 0.01 * 0.005;;
    Q(3,3)=0.1 * 0.05;
//
//    // observation model covariance
//    R = Eigen::Matrix3f::Identity();
//    R(0,0)=2.050;
//    R(1,1)=2.050;
////    R(2,2)=10.050;

// Motional model covariance
//    Eigen::Matrix4f Q = Eigen::Matrix4f::Identity();
//    Q(0,0)=0.1 * 0.1;
//    Q(1,1)=0.1 * 0.1;
//    Q(2,2)=(1.0/180 * M_PI) * (1.0/180 * M_PI);
//    Q(3,3)=0.1 * 0.1;

    // Observation model covariance
    Eigen::Matrix2f  R = Eigen::Matrix2f::Identity();
    R(0,0)=1.0;
    R(1,1)=1.0;

}



