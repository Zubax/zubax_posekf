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

using Scalar = double;
using Quaternion = Eigen::Quaternion<Scalar>;
template <int Rows, int Cols> using Matrix = Eigen::Matrix<Scalar, Rows, Cols>;
template <int Size> using Vector = Matrix<Size, 1>;
using Vector3 = Vector<3>;
using Matrix3 = Matrix<3, 3>;

inline Quaternion quaternionFromEuler(const Vector3& roll_pitch_yaw)
{
    const Scalar ai = roll_pitch_yaw[0];
    const Scalar aj = roll_pitch_yaw[1];
    const Scalar ak = roll_pitch_yaw[2];

    const Scalar ci = std::cos(ai / 2.0);
    const Scalar si = std::sin(ai / 2.0);
    const Scalar cj = std::cos(aj / 2.0);
    const Scalar sj = std::sin(aj / 2.0);
    const Scalar ck = std::cos(ak / 2.0);
    const Scalar sk = std::sin(ak / 2.0);

    const Scalar cc = ci * ck;
    const Scalar cs = ci * sk;
    const Scalar sc = si * ck;
    const Scalar ss = si * sk;

    return Quaternion(
        cj * cc + sj * ss,   // w
        cj * sc - sj * cs,   // x
        cj * ss + sj * cc,   // y
        cj * cs - sj * sc);  // z
}

inline Vector3 quaternionToEuler(const Quaternion& q)
{
    return q.toRotationMatrix().eulerAngles(0, 1, 2);
}

inline Quaternion computeDeltaQuaternion(const Quaternion& from, const Quaternion& to)
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
    Vector<10> x_;
    Matrix<10, 10> P_;
    Matrix<10, 10> Q_;

    const Scalar AccelCovMult = 1000.0;

    Scalar state_timestamp_ = 0.0;
    bool initialized_ = false;

    Vector3 accel_;       ///< This is not included in the state vector
    Matrix3 accel_cov_;   ///< Ditto

    Quaternion getQuat() const  { return Quaternion(x_(0, 0), x_(1, 0), x_(2, 0), x_(3, 0)); }
    Vector3 getAngVel() const   { return Vector3(x_(4, 0), x_(5, 0), x_(6, 0)); }
    Vector3 getGyroBias() const { return Vector3(x_(7, 0), x_(8, 0), x_(9, 0)); }

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

    Matrix<10, 10> computeStateTransitionJacobian(const Scalar dtf) const
    {
        const Scalar qw = x_(0, 0);
        const Scalar qx = x_(1, 0);
        const Scalar qy = x_(2, 0);
        const Scalar qz = x_(3, 0);

        const Scalar wlx = x_(4, 0);
        const Scalar wly = x_(5, 0);
        const Scalar wlz = x_(6, 0);

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

    Matrix<3, 10> computeAccelMeasurementJacobian() const
    {
        const Scalar qw = x_(0, 0);
        const Scalar qx = x_(1, 0);
        const Scalar qy = x_(2, 0);
        const Scalar qz = x_(3, 0);

        using namespace mathematica;
        return List(List(2 * qy, 2 * qz, 2 * qw, 2 * qx, 0, 0, 0, 0, 0, 0),
                    List(-2 * qx, -2 * qw, 2 * qz, 2 * qy, 0, 0, 0, 0, 0, 0),
                    List(2 * qw, -2 * qx, -2 * qy, 2 * qz, 0, 0, 0, 0, 0, 0));
    }

    Matrix<3, 10> computeGyroMeasurementJacobian() const
    {
        using namespace mathematica;
        return List(List(0, 0, 0, 0, 1, 0, 0, 1, 0, 0),
                    List(0, 0, 0, 0, 0, 1, 0, 0, 1, 0),
                    List(0, 0, 0, 0, 0, 0, 1, 0, 0, 1));
    }

    Vector3 predictAccelMeasurement() const
    {
        const Scalar qw = x_(0, 0);
        const Scalar qx = x_(1, 0);
        const Scalar qy = x_(2, 0);
        const Scalar qz = x_(3, 0);

        using namespace mathematica;
        return List(List(2 * qw * qy + 2 * qx * qz),
                    List(-2 * qw * qx + 2 * qy * qz),
                    List(Power(qw, 2) - Power(qx, 2) - Power(qy, 2) + Power(qz, 2)));
    }

    Vector3 predictGyroMeasurement() const
    {
        const Scalar wlx = x_(4, 0);
        const Scalar wly = x_(5, 0);
        const Scalar wlz = x_(6, 0);

        const Scalar bwlx = x_(7, 0);
        const Scalar bwly = x_(8, 0);
        const Scalar bwlz = x_(9, 0);

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
        Matrix<decltype(Q_)::RowsAtCompileTime, 1> Q_diag;
        Q_diag.setZero();
        Q_diag <<
            0.0,   0.1,   0.1,   0.1,   // q (w x y z)
            0.1,   0.1,   0.1,          // angvel
            0.001, 0.001, 0.001;        // gyro bias
        Q_ = Q_diag.asDiagonal();

        accel_.setZero();
    }

    bool isInitialized() const { return initialized_; }

    void initialize(Scalar timestamp, const Quaternion& orientation, const Matrix3& orientation_cov)
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

    void performTimeUpdate(Scalar timestamp)
    {
        ROS_ASSERT(initialized_);
        ROS_ASSERT(timestamp > 0);

        /*
         * Compute and check dt
         */
        const Scalar dtf = timestamp - state_timestamp_;
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
        const Quaternion delta_quat = quaternionFromEuler(getAngVel() * dtf);
        const Quaternion new_q = (getQuat() * delta_quat).normalized();

        x_(0, 0) = new_q.w();
        x_(1, 0) = new_q.x();
        x_(2, 0) = new_q.y();
        x_(3, 0) = new_q.z();

        /*
         * Predict covariance
         */
        const Matrix<10, 10> F = computeStateTransitionJacobian(dtf);
        P_ = F * P_ * F.transpose() + Q_;

        normalizeAndCheck();
    }

    void performAccelUpdate(Scalar timestamp, const Vector3& accel, const Matrix3& cov)
    {
        ROS_ASSERT(initialized_);
        ROS_ASSERT(timestamp > 0);

        state_timestamp_ = timestamp;

        accel_ = accel;
        accel_cov_ = cov;

        const Matrix3 R = cov * AccelCovMult;

        const Vector3 y = accel.normalized() - predictAccelMeasurement();

        const Matrix<3, 10> H = computeAccelMeasurementJacobian();

        const Matrix3 S = H * P_ * H.transpose() + R;

        const auto K = static_cast<Matrix<10, 3> >(P_ * H.transpose() * S.inverse());

        x_ = x_ + K * y;

        const Matrix<10, 10> IKH = decltype(P_)::Identity() - K * H;
        P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();

        debug_pub_.publish("K_accel", K);

        normalizeAndCheck();
    }

    void performGyroUpdate(Scalar timestamp, const Vector3& angvel, const Matrix3& cov)
    {
        ROS_ASSERT(initialized_);
        ROS_ASSERT(timestamp > 0);

        state_timestamp_ = timestamp;

        const Matrix3 R = cov;

        const Vector3 y = angvel - predictGyroMeasurement();

        const Matrix<3, 10> H = computeGyroMeasurementJacobian();

        const Matrix3 S = H * P_ * H.transpose() + R;

        const auto K = static_cast<Matrix<10, 3> >(P_ * H.transpose() * S.inverse());

        x_ = x_ + K * y;

        const Matrix<10, 10> IKH = decltype(P_)::Identity() - K * H;
        P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();

        debug_pub_.publish("K_gyro", K);

        normalizeAndCheck();
    }

    Scalar getTimestamp() const { return state_timestamp_; }

    std::pair<Quaternion, Matrix3> getOutputOrientation() const
    {
        /*
         * TODO: Proper covariance conversion (ref. S. Weiss)
         */
        return { getQuat(), P_.block<3, 3>(1, 1) };
    }

    std::pair<Vector3, Matrix3> getOutputAngularVelocity() const
    {
        return { x_.block<3, 1>(4, 0), P_.block<3, 3>(4, 4) };
    }

    std::pair<Vector3, Matrix3> getOutputAcceleration() const
    {
        return { accel_, accel_cov_ };
    }
};

}
