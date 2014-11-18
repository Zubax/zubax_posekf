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
#include "linear_algebra.hpp"
#include "exception.hpp"
#include "state_vector_autogenerated.hpp"

namespace zubax_posekf
{
/**
 * Filter output - twist; compatible with geometry_msgs/TwistWithCovariance.
 * - Velocity of IMU in the world frame, transformed into the IMU frame
 * - Angular velocity of IMU in the IMU frame
 * - Covariance: x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis
 */
struct Twist
{
    Vector3 linear;
    Vector3 angular;
    Matrix<6, 6> linear_angular_cov;

    Twist()
    {
        linear.setZero();
        angular.setZero();
        linear_angular_cov.setZero();
    }
};

/**
 * Filter output - pose; compatible with geometry_msgs/PoseWithCovariance.
 * - Position of IMU in the world frame
 * - Orientation of IMU in the world frame
 * - Covariance: x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis
 */
struct Pose
{
    Vector3 position;
    Quaternion orientation;
    Matrix<6, 6> position_orientation_cov;

    Pose()
    {
        position.setZero();
        orientation.setIdentity();
        position_orientation_cov.setZero();
    }
};

/**
 * The business end.
 */
class Filter
{
    DebugPublisher debug_pub_;

    StateVector state_;
    Matrix<StateVector::Size, StateVector::Size> P_;
    Matrix<StateVector::Size, StateVector::Size> Q_;

    const Scalar MaxGyroDrift = 0.1;
    const Scalar MaxAccelDrift = 2.0;
    const Scalar MaxCovariance = 1e9;
    const Scalar MinVariance = 1e-9;

    Scalar state_timestamp_ = 0.0;
    bool initialized_ = false;

    static Vector3 constrainDrift(Vector3 vec, const Scalar limit, const char* name)
    {
        for (int i = 0; i < 3; i++)
        {
            if (!checkRangeAndConstrainSymmetric(vec[i], limit))
            {
                ROS_WARN_THROTTLE(1, "Drift is too high [%s]", name);
            }
        }
        return vec;
    }

    void normalizeAndCheck()
    {
        state_.normalize();

        state_.bw(constrainDrift(state_.bw(), MaxGyroDrift, "gyro"));

        state_.ba(constrainDrift(state_.ba(), MaxAccelDrift, "accel"));

        // P validation
        if (!validateAndFixCovarianceMatrix(P_, MaxCovariance, MinVariance))
        {
            ROS_ERROR_STREAM_THROTTLE(1, "Matrix P has been fixed\n" << P_.format(Eigen::IOFormat(2)));
        }

        debug_pub_.publish("x", state_.x);
        debug_pub_.publish("P", P_);
        debug_pub_.publish("Q", Q_);

        enforce("Non-finite states",
                std::isfinite(state_.x.sum()) &&
                std::isfinite(P_.sum()) &&
                std::isfinite(Q_.sum()));

        enforce("Invalid timestamp", std::isfinite(state_timestamp_) && (state_timestamp_ > 0.0));
    }

    template <int NumDims>
    void performMeasurementUpdate(const Vector<NumDims>& y,
                                  const Matrix<NumDims, NumDims>& R,
                                  const Matrix<NumDims, StateVector::Size>& H,
                                  const char* const source_name)
    {
        ROS_ASSERT(initialized_);

        /*
         * Ensure that R is symmetric, then validate
         */
        const Matrix<NumDims, NumDims> R_sym = 0.5 * (R + R.transpose());
        {
            bool R_sym_ok = true;
            for (int i = 0; i < NumDims; i++)
            {
                if (R_sym(i, i) <= 0)
                {
                    R_sym_ok = false;
                    break;
                }
            }

            if (!R_sym_ok || !std::isfinite(R_sym.sum()))
            {
                std::ostringstream os;
                os << "Measurement R_sym [" << source_name << "]:\n" << R_sym;
                throw Exception(os.str());
            }
        }

        /*
         * Compute S inverse with validation
         */
        Matrix<NumDims, NumDims> S_inv;
        {
            const Matrix<NumDims, NumDims> S = H * P_ * H.transpose() + R_sym;
            bool s_is_invertible = false;
            S.computeInverseWithCheck(S_inv, s_is_invertible);
            if (!s_is_invertible)
            {
                std::ostringstream os;
                os << "S is not invertible (numerical failure?) [" << source_name << "]:\n" << S
                    << "\nR_sym:\n" << R_sym;
                throw Exception(os.str());
            }
        }

        /*
         * Kalman update equations
         * Joseph form is used instead of the simplified form to improve numerical stability
         */
        const auto K = static_cast<Matrix<StateVector::Size, NumDims> >(P_ * H.transpose() * S_inv);

        state_.x = state_.x + K * y;

        const Matrix<StateVector::Size, StateVector::Size> IKH = decltype(P_)::Identity() - K * H;
        P_ = IKH * P_ * IKH.transpose() + K * R_sym * K.transpose();

        normalizeAndCheck();

//        std::cout << "P:\n" << P_.format(Eigen::IOFormat(2)) << "\n" << std::endl;
//        std::cout << "x:\n" << state_.x.transpose().format(Eigen::IOFormat(2)) << "\n" << std::endl;
    }

public:
    Filter()
    {
        // Initial state - zero everything, null rotations
        state_.qwi(Quaternion(1, 0, 0, 0));
        state_.qvw(Quaternion(1, 0, 0, 0));

        // Initial P
        P_.setZero();
        P_ = StateVector::Pinitdiag().asDiagonal();

        // Initial Q
        // TODO: runtime Q estimation
        Q_ = StateVector::Qmindiag().asDiagonal();
    }

    bool isInitialized() const { return initialized_; }

    void initialize(Scalar timestamp, const Quaternion& orientation)
    {
        ROS_ASSERT(!initialized_);
        initialized_ = true;

        state_timestamp_ = timestamp;

        state_.qwi(orientation);

        normalizeAndCheck();

        ROS_INFO_STREAM("Initial P:\n" << P_.format(Eigen::IOFormat(2)));
        ROS_INFO_STREAM("Initial Q:\n" << Q_.format(Eigen::IOFormat(2)));
        ROS_INFO_STREAM("Initial x:\n" << state_.x.transpose().format(Eigen::IOFormat(2)));
    }

    void performTimeUpdate(Scalar timestamp)
    {
        ROS_ASSERT(initialized_);
        ROS_ASSERT(timestamp > 0);

        /*
         * Compute and check dt
         */
        const Scalar dt = timestamp - state_timestamp_;
        debug_pub_.publish("dt", dt);
        if (dt <= 0)
        {
            ROS_WARN_THROTTLE(1, "Time update: Nonpositive dt [%f]", dt);
            return;
        }
        state_timestamp_ = timestamp;

        /*
         * Update the state vector and process covariance matrix
         */
        const auto F = state_.F(dt);

        state_.x = state_.f(dt);

        P_ = F * P_ * F.transpose() + Q_ * dt;

        normalizeAndCheck();

        // TODO: Commit point - add new X to the state history
    }

    void performAccelUpdate(const Vector3& accel, const Matrix3& cov)
    {
        const Vector3 y = accel - state_.hacc();
        performMeasurementUpdate(y, cov, state_.Hacc(), "acc");
    }

    void performGyroUpdate(const Vector3& angvel, const Matrix3& cov)
    {
        const Vector3 y = angvel - state_.hgyro();
        performMeasurementUpdate(y, cov, state_.Hgyro(), "gyro");
    }

    void performGNSSPosUpdate(const Vector3& pos, const Matrix3& cov)
    {
        const Vector3 y = pos - state_.hgnsspos();
        performMeasurementUpdate(y, cov, state_.Hgnsspos(), "gnsspos");
    }

    void performGNSSVelUpdate(const Vector3& vel, const Matrix3& cov)
    {
        const Vector3 y = vel - state_.hgnssvel();
        performMeasurementUpdate(y, cov, state_.Hgnssvel(), "gnssvel");
    }

    void performVisPosUpdate(const Vector3& pos, const Matrix3& cov)
    {
        const Vector3 y = pos - state_.hvispos();
        performMeasurementUpdate(y, cov, state_.Hvispos(), "vispos");
    }

    void performVisAttUpdate(const Quaternion& z, const Matrix3& cov)
    {
        /*
         * Residual computation
         */
        const Quaternion h = state_.hvisatt();

        const Quaternion yq = z * h.inverse();  // Weiss 2012, eq. 3.45 ~ 3.46

        Vector<4> y;
        y[0] = yq.w();
        y[1] = yq.x();
        y[2] = yq.y();
        y[3] = yq.z();

        /*
         * Covariance transformation
         */
        const Matrix<4, 3> G = quaternionFromEulerJacobian(quaternionToEuler(z));
        const auto R = static_cast<Matrix<4, 4> >(G * cov * G.transpose());

        performMeasurementUpdate(y, R, state_.Hvisatt(), "visatt");

        debug_pub_.publish("visatt_R", R);
    }

    Scalar getTimestamp() const { return state_timestamp_; }

    /**
     * Pose, compatible with geometry_msgs/PoseWithCovariance
     */
    Pose getOutputPose() const
    {
        Pose out;

        out.position = state_.pwi();
        out.orientation = state_.qwi();

        const Matrix<3, 4> G = quaternionToEulerJacobian(out.orientation);
        const Matrix4 C = P_.block<4, 4>(StateVector::Idx::qwiw, StateVector::Idx::qwiw);

        out.position_orientation_cov.block<3, 3>(0, 0) = P_.block<3, 3>(StateVector::Idx::pwix, StateVector::Idx::pwix);
        out.position_orientation_cov.block<3, 3>(3, 3) = G * C * G.transpose();

        return out;
    }

    /**
     * Twist, compatible with geometry_msgs/TwistWithCovariance
     */
    Twist getOutputTwistInIMUFrame() const
    {
        Twist out;
        out.linear = rotateVectorByQuaternion(state_.vwi(), state_.qwi());  // World --> IMU
        out.angular = state_.w();

        const Matrix3 G = rotateVectorByQuaternionJacobian(state_.qwi());
        const Matrix3 C = P_.block<3, 3>(StateVector::Idx::vwix, StateVector::Idx::vwix);

        out.linear_angular_cov.block<3, 3>(0, 0) = G * C * G.transpose();
        out.linear_angular_cov.block<3, 3>(3, 3) = P_.block<3, 3>(StateVector::Idx::wx,   StateVector::Idx::wx);

        return out;
    }

    /**
     * Gravity compensated acceleration in IMU frame with covariance
     */
    std::pair<Vector3, Matrix3> getOutputAcceleration() const
    {
        return { state_.a(), P_.block<3, 3>(StateVector::Idx::ax, StateVector::Idx::ax) };
    }
};

}
