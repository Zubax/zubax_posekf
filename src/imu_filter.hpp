/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Please refer to the file LICENSE for terms and conditions.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <eigen3/Eigen/Eigen>
#include <cmath>
#include "mathematica.hpp"
#include "debug_publisher.hpp"

namespace zubax_posekf
{

inline Eigen::Quaterniond quaternionFromEuler(const Eigen::Vector3d& roll_pitch_yaw)
{
    const double ai = roll_pitch_yaw[0];
    const double aj = roll_pitch_yaw[1];
    const double ak = roll_pitch_yaw[2];

    const double ci = std::cos(ai / 2.0);
    const double si = std::sin(ai / 2.0);
    const double cj = std::cos(aj / 2.0);
    const double sj = std::sin(aj / 2.0);
    const double ck = std::cos(ak / 2.0);
    const double sk = std::sin(ak / 2.0);

    const double cc = ci * ck;
    const double cs = ci * sk;
    const double sc = si * ck;
    const double ss = si * sk;

    return Eigen::Quaterniond(
        cj * cc + sj * ss,   // w
        cj * sc - sj * cs,   // x
        cj * ss + sj * cc,   // y
        cj * cs - sj * sc);  // z
}

inline Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond& q)
{
    return q.toRotationMatrix().eulerAngles(0, 1, 2);
}

inline Eigen::Quaterniond computeDeltaQuaternion(const Eigen::Quaterniond& from, const Eigen::Quaterniond& to)
{
    return to.normalized() * from.inverse();
}

class IMUFilter
{
    DebugPublisher debug_pub_;

    /*
     * State vector:
     *  q - earth to body quaternion (w x y z)
     *  w - true angular velocity in body frame
     *  bw - gyro bias in body frame
     */
    Eigen::Matrix<double, 10, 1> x_;
    Eigen::Matrix<double, 10, 10> P_;
    Eigen::Matrix<double, 10, 10> Q_;

    double state_timestamp_ = 0.0;
    bool initialized_ = false;

    Eigen::Vector3d accel_;       ///< This is not included in the state vector
    Eigen::Matrix3d accel_cov_;   ///< Ditto

    Eigen::Quaterniond getQuat() const  { return Eigen::Quaterniond(x_(0, 0), x_(1, 0), x_(2, 0), x_(3, 0)); }
    Eigen::Vector3d getAngVel() const   { return Eigen::Vector3d(x_(4, 0), x_(5, 0), x_(6, 0)); }
    Eigen::Vector3d getGyroBias() const { return Eigen::Vector3d(x_(7, 0), x_(8, 0), x_(9, 0)); }

    void normalizeAndCheck()
    {
        const auto q = getQuat().normalized();
        x_(0, 0) = q.w();
        x_(1, 0) = q.x();
        x_(2, 0) = q.y();
        x_(3, 0) = q.z();

        P_ = 0.5 * (P_ + P_.transpose());  // Make sure P stays symmetric

        debug_pub_.publish("x", x_);
        debug_pub_.publish("P", P_);
        debug_pub_.publish("Q", Q_);

        ROS_ASSERT(std::isfinite(x_.sum()));
        ROS_ASSERT(std::isfinite(P_.sum()));
        ROS_ASSERT(std::isfinite(Q_.sum()));

        for (int i = 0; i < P_.rows(); i++)
        {
            if (P_(i, i) <= 0.0)
            {
                ROS_FATAL_STREAM("Invalid P:\n" << P_ << "\nx:\n" << x_);
                ROS_ISSUE_BREAK();
            }
        }

        ROS_ASSERT(std::isfinite(state_timestamp_) && (state_timestamp_ > 0.0));
    }

    Eigen::Matrix<double, 10, 10> computeStateTransitionJacobian(const double dtf) const
    {
        const double qw = x_(0, 0);
        const double qx = x_(1, 0);
        const double qy = x_(2, 0);
        const double qz = x_(3, 0);

        const double wlx = x_(4, 0);
        const double wly = x_(5, 0);
        const double wlz = x_(6, 0);

        using namespace mathematica;
        return List(
            List(1, -(dtf * wlx) / 2., -(dtf * wly) / 2., -(dtf * wlz) / 2., -(dtf * qx) / 2., -(dtf * qy) / 2.,
                 -(dtf * qz) / 2., 0, 0, 0),
            List((dtf * wlx) / 2., 1, (dtf * wlz) / 2., -(dtf * wly) / 2., (dtf * qw) / 2., -(dtf * qz) / 2.,
                 (dtf * qy) / 2., 0, 0, 0),
            List((dtf * wly) / 2., -(dtf * wlz) / 2., 1, (dtf * wlx) / 2., (dtf * qz) / 2., (dtf * qw) / 2.,
                 -(dtf * qx) / 2., 0, 0, 0),
            List((dtf * wlz) / 2., (dtf * wly) / 2., -(dtf * wlx) / 2., 1, -(dtf * qy) / 2., (dtf * qx) / 2.,
                 (dtf * qw) / 2., 0, 0, 0),
            List(0, 0, 0, 0, 1, 0, 0, 0, 0, 0),
            List(0, 0, 0, 0, 0, 1, 0, 0, 0, 0),
            List(0, 0, 0, 0, 0, 0, 1, 0, 0, 0),
            List(0, 0, 0, 0, 0, 0, 0, 1, 0, 0),
            List(0, 0, 0, 0, 0, 0, 0, 0, 1, 0),
            List(0, 0, 0, 0, 0, 0, 0, 0, 0, 1));
    }

    Eigen::Matrix<double, 3, 10> computeAccelMeasurementJacobian() const
    {
        const double qw = x_(0, 0);
        const double qx = x_(1, 0);
        const double qy = x_(2, 0);
        const double qz = x_(3, 0);

        using namespace mathematica;
        return List(List(2 * qy, 2 * qz, 2 * qw, 2 * qx, 0, 0, 0, 0, 0, 0),
                    List(-2 * qx, -2 * qw, 2 * qz, 2 * qy, 0, 0, 0, 0, 0, 0),
                    List(2 * qw, -2 * qx, -2 * qy, 2 * qz, 0, 0, 0, 0, 0, 0));
    }

    Eigen::Matrix<double, 3, 10> computeGyroMeasurementJacobian() const
    {
        using namespace mathematica;
        return List(List(0, 0, 0, 0, 1, 0, 0, 1, 0, 0),
                    List(0, 0, 0, 0, 0, 1, 0, 0, 1, 0),
                    List(0, 0, 0, 0, 0, 0, 1, 0, 0, 1));
    }

    Eigen::Vector3d predictAccelMeasurement() const
    {
        const double qw = x_(0, 0);
        const double qx = x_(1, 0);
        const double qy = x_(2, 0);
        const double qz = x_(3, 0);

        using namespace mathematica;
        return List(List(2 * qw * qy + 2 * qx * qz),
                    List(-2 * qw * qx + 2 * qy * qz),
                    List(Power(qw, 2) - Power(qx, 2) - Power(qy, 2) + Power(qz, 2)));
    }

    Eigen::Vector3d predictGyroMeasurement() const
    {
        const double wlx = x_(4, 0);
        const double wly = x_(5, 0);
        const double wlz = x_(6, 0);

        const double bwlx = x_(7, 0);
        const double bwly = x_(8, 0);
        const double bwlz = x_(9, 0);

        using namespace mathematica;
        return List(List(bwlx + wlx),
                    List(bwly + wly),
                    List(bwlz + wlz));
    }

public:
    IMUFilter()
    {
        x_.setZero();
        P_.setZero();

        // TODO: runtime Q estimation
        Eigen::Matrix<double, decltype(Q_)::RowsAtCompileTime, 1> Q_diag;
        Q_diag.setZero();
        Q_diag <<
            0.0,   0.1,   0.1,   0.1,   // q (w x y z)
            0.1,   0.1,   0.1,          // angvel
            0.001, 0.001, 0.001;        // gyro bias
        Q_ = Q_diag.asDiagonal();

        accel_.setZero();
    }

    bool isInitialized() const { return initialized_; }

    void initialize(double timestamp, const Eigen::Quaterniond& orientation, const Eigen::Matrix3d& orientation_cov)
    {
        ROS_ASSERT(!initialized_);
        initialized_ = true;

        state_timestamp_ = timestamp;

        x_.setZero();
        x_(0, 0) = orientation.w();
        x_(1, 0) = orientation.x();
        x_(2, 0) = orientation.y();
        x_(3, 0) = orientation.z();

        P_.setIdentity();
        P_.block<3, 3>(0, 0) = orientation_cov;

        normalizeAndCheck();

        ROS_INFO_STREAM("Initial x:\n" << x_);
        ROS_INFO_STREAM("Initial P:\n" << P_);
    }

    void performTimeUpdate(double timestamp)
    {
        ROS_ASSERT(initialized_);
        ROS_ASSERT(timestamp > 0);

        /*
         * Compute and check dt
         */
        const double dtf = timestamp - state_timestamp_;
        debug_pub_.publish("dtf", dtf);
        if (dtf <= 0)
        {
            ROS_WARN("Time update: Nonpositive dt [%f]", dtf);
            return;
        }
        state_timestamp_ = timestamp;

        /*
         * Predict state
         */
        const Eigen::Quaterniond delta_quat = quaternionFromEuler(getAngVel() * dtf);
        const Eigen::Quaterniond new_q = (getQuat() * delta_quat).normalized();

        x_(0, 0) = new_q.w();
        x_(1, 0) = new_q.x();
        x_(2, 0) = new_q.y();
        x_(3, 0) = new_q.z();

        /*
         * Predict covariance
         */
        const Eigen::Matrix<double, 10, 10> F = computeStateTransitionJacobian(dtf);
        P_ = F * P_ * F.transpose() + Q_;

        normalizeAndCheck();
    }

    void performAccelUpdate(double timestamp, const Eigen::Vector3d& accel, const Eigen::Matrix3d& cov)
    {
        ROS_ASSERT(initialized_);
        ROS_ASSERT(timestamp > 0);

        state_timestamp_ = timestamp;

        accel_ = accel;
        accel_cov_ = cov;

        const Eigen::Vector3d y = accel.normalized() - predictAccelMeasurement();

        const Eigen::Matrix<double, 3, 10> H = computeAccelMeasurementJacobian();

        const Eigen::Matrix3d S = H * P_ * H.transpose() + cov;

        const auto K = static_cast<Eigen::Matrix<double, 10, 3> >(P_ * H.transpose() * S.inverse());

        x_ = x_ + K * y;

        P_ = (decltype(P_)::Identity() - K * H) * P_;

        debug_pub_.publish("K_accel", K);

        normalizeAndCheck();
    }

    void performGyroUpdate(double timestamp, const Eigen::Vector3d& angvel, const Eigen::Matrix3d& cov)
    {
        ROS_ASSERT(initialized_);
        ROS_ASSERT(timestamp > 0);

        state_timestamp_ = timestamp;

        const Eigen::Vector3d y = angvel - predictGyroMeasurement();

        const Eigen::Matrix<double, 3, 10> H = computeGyroMeasurementJacobian();

        const Eigen::Matrix3d S = H * P_ * H.transpose() + cov;

        const auto K = static_cast<Eigen::Matrix<double, 10, 3> >(P_ * H.transpose() * S.inverse());

        x_ = x_ + K * y;

        P_ = (decltype(P_)::Identity() - K * H) * P_;    // TODO: Proper covariance propagation

        debug_pub_.publish("K_gyro", K);

        normalizeAndCheck();
    }

    double getTimestamp() const { return state_timestamp_; }

    std::pair<Eigen::Quaterniond, Eigen::Matrix3d> getOutputOrientation() const
    {
        /*
         * TODO: Proper covariance conversion (ref. S. Weiss)
         */
        return { getQuat(), P_.block<3, 3>(1, 1) };
    }

    std::pair<Eigen::Vector3d, Eigen::Matrix3d> getOutputAngularVelocity() const
    {
        return { x_.block<3, 1>(4, 0), P_.block<3, 3>(4, 4) };
    }

    std::pair<Eigen::Vector3d, Eigen::Matrix3d> getOutputAcceleration() const
    {
        return { accel_, accel_cov_ };
    }
};

}
