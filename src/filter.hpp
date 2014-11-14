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

namespace zubax_posekf
{

class IMUFilter
{
    DebugPublisher debug_pub_;

    /*
     * State vector:
     *  q  - earth to body quaternion (w x y z)
     *  wl - true angular velocity in body frame
     *  al - true acceleration in body frame
     *  bw - gyro bias
     *  ba - accel bias
     */
    static constexpr int NumStates = 16;
    Vector<NumStates> x_;
    Matrix<NumStates, NumStates> P_;
    Matrix<NumStates, NumStates> Q_;

    const Scalar MaxGyroDrift = 0.1;
    const Scalar MaxAccelDrift = 2.0;
    const Scalar MaxCovariance = 1e6;

    Scalar state_timestamp_ = 0.0;
    bool initialized_ = false;

    Quaternion getQuat() const  { return Quaternion(x_[0], x_[1], x_[2], x_[3]); }

    Vector3 getBodyAngVel() const { return Vector3(x_[4], x_[5], x_[6]); }
    Vector3 getBodyAccel()  const { return Vector3(x_[7], x_[8], x_[9]); }

    Vector3 getGyroBias()  const { return Vector3(x_[10], x_[11], x_[12]); }
    Vector3 getAccelBias() const { return Vector3(x_[13], x_[14], x_[15]); }

    void normalizeAndCheck()
    {
        // Quaternion normalization
        const auto q = getQuat().normalized();
        x_[0] = q.w();
        x_[1] = q.x();
        x_[2] = q.y();
        x_[3] = q.z();

        // Gyro drift constrain
        for (int i = 10; i <= 12; i++)
        {
            if (!checkRangeAndConstrainSymmetric(x_[i], MaxGyroDrift))
            {
                ROS_WARN_THROTTLE(1, "Gyro drift is too high");
            }
        }

        // Accel drift constrain
        for (int i = 13; i <= 15; i++)
        {
            if (!checkRangeAndConstrainSymmetric(x_[i], MaxAccelDrift))
            {
                ROS_WARN_THROTTLE(1, "Accel drift is too high");
            }
        }

        // P validation
        if (!validateAndFixCovarianceMatrix(P_, MaxCovariance, 1e-9))
        {
            ROS_ERROR_STREAM_THROTTLE(1, "Matrix P has been fixed\n" << P_);
        }

        debug_pub_.publish("x", x_);
        debug_pub_.publish("P", P_);
        debug_pub_.publish("Q", Q_);

        enforce("Non-finite states", std::isfinite(x_.sum()) && std::isfinite(P_.sum()) && std::isfinite(Q_.sum()));
        enforce("Invalid timestamp", std::isfinite(state_timestamp_) && (state_timestamp_ > 0.0));
    }

    template <int NumDims>
    void performMeasurementUpdate(Scalar timestamp,
                                  const Vector<NumDims>& y,
                                  const Matrix<NumDims, NumDims>& R,
                                  const Matrix<NumDims, NumStates>& H)
    {
        ROS_ASSERT(initialized_);
        ROS_ASSERT(timestamp > 0);

        state_timestamp_ = timestamp;

        const Matrix<NumDims, NumDims> R_sym = 0.5 * (R + R.transpose());  // Ensure that R is symmetric

        Matrix<NumDims, NumDims> S_inv;
        {
            const Matrix<NumDims, NumDims> S = H * P_ * H.transpose() + R_sym;
            bool s_is_invertible = false;
            S.computeInverseWithCheck(S_inv, s_is_invertible);
            enforce("S is not invertible", s_is_invertible);
        }

        const auto K = static_cast<Matrix<NumStates, NumDims> >(P_ * H.transpose() * S_inv);

        x_ = x_ + K * y;

        // Joseph form is used instead of the simplified form to improve numerical stability
        const Matrix<NumStates, NumStates> IKH = decltype(P_)::Identity() - K * H;
        P_ = IKH * P_ * IKH.transpose() + K * R_sym * K.transpose();

        normalizeAndCheck();
    }

public:
    IMUFilter()
    {
        x_.setZero();
        P_.setZero();

        // TODO: runtime Q estimation
        const Scalar QVarQuat      = 1e-6;
        const Scalar QVarAngVel    = 1e+0;
        const Scalar QVarAccel     = 1e+0;
        const Scalar QVarGyroBias  = 0;     // Assuming that biases are constant
        const Scalar QVarAccelBias = 0;

        Vector<NumStates> Q_diag;
        Q_diag.setZero();
        Q_diag <<
            QVarQuat,      QVarQuat,      QVarQuat,      QVarQuat,
            QVarAngVel,    QVarAngVel,    QVarAngVel,
            QVarAccel,     QVarAccel,     QVarAccel,
            QVarGyroBias,  QVarGyroBias,  QVarGyroBias,
            QVarAccelBias, QVarAccelBias, QVarAccelBias;

        Q_ = Q_diag.asDiagonal();
    }

    bool isInitialized() const { return initialized_; }

    void initialize(Scalar timestamp, const Quaternion& orientation, const Matrix3& orientation_cov)
    {
        (void)orientation_cov;
        ROS_ASSERT(!initialized_);
        initialized_ = true;

        state_timestamp_ = timestamp;

        x_.setZero();
        x_[0] = orientation.w();
        x_[1] = orientation.x();
        x_[2] = orientation.y();
        x_[3] = orientation.z();

        P_ = decltype(P_)::Identity() * 1e3;
        P_.block<6, 6>(4, 4) = Matrix<6, 6>::Identity() * 1e-1;
        P_.block<6, 6>(10, 10) = Matrix<6, 6>::Identity() * 1e-3;

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
            ROS_ERROR("Time update: Nonpositive dt [%f]", dtf);
            return;
        }
        state_timestamp_ = timestamp;

        /*
         * Predict state
         */
        const Quaternion delta_quat = quaternionFromEuler(getBodyAngVel() * dtf);
        const Quaternion new_q = (getQuat() * delta_quat).normalized();

        x_[0] = new_q.w();
        x_[1] = new_q.x();
        x_[2] = new_q.y();
        x_[3] = new_q.z();

        /*
         * Predict covariance
         */
        const Scalar qw = x_[0];
        const Scalar qx = x_[1];
        const Scalar qy = x_[2];
        const Scalar qz = x_[3];

        const Scalar wlx = x_[4];
        const Scalar wly = x_[5];
        const Scalar wlz = x_[6];

        using namespace mathematica;
        const auto F = List(
            List(1, -(dtf * wlx) / 2., -(dtf * wly) / 2., -(dtf * wlz) / 2., -(dtf * qx) / 2., -(dtf * qy) / 2.,
                 -(dtf * qz) / 2., 0, 0, 0, 0, 0, 0, 0, 0, 0),
            List((dtf * wlx) / 2., 1, (dtf * wlz) / 2., -(dtf * wly) / 2., (dtf * qw) / 2., -(dtf * qz) / 2.,
                 (dtf * qy) / 2., 0, 0, 0, 0, 0, 0, 0, 0, 0),
            List((dtf * wly) / 2., -(dtf * wlz) / 2., 1, (dtf * wlx) / 2., (dtf * qz) / 2., (dtf * qw) / 2.,
                 -(dtf * qx) / 2., 0, 0, 0, 0, 0, 0, 0, 0, 0),
            List((dtf * wlz) / 2., (dtf * wly) / 2., -(dtf * wlx) / 2., 1, -(dtf * qy) / 2., (dtf * qx) / 2.,
                 (dtf * qw) / 2., 0, 0, 0, 0, 0, 0, 0, 0, 0),
            List(0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
            List(0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
            List(0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0),
            List(0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0),
            List(0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
            List(0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0),
            List(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0),
            List(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0),
            List(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0),
            List(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0),
            List(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0),
            List(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1));

        P_ = F * P_ * F.transpose() + Q_;

        normalizeAndCheck();
    }

    void performAccelUpdate(Scalar timestamp, const Vector3& accel, const Matrix3& cov)
    {
        enforce("Accel R", cov.norm() > 0);

        const Scalar qw = x_[0];
        const Scalar qx = x_[1];
        const Scalar qy = x_[2];
        const Scalar qz = x_[3];

        const Scalar alx = x_[7];
        const Scalar aly = x_[8];
        const Scalar alz = x_[9];

        const Scalar bax = x_[13];
        const Scalar bay = x_[14];
        const Scalar baz = x_[15];

        using namespace mathematica;

        const Vector3 h = List(List(alx + bax - 19.6133 * qw * qy + 19.6133 * qx * qz),
                               List(aly + bay + 19.6133 * qw * qx + 19.6133 * qy * qz),
                               List(alz + baz + 9.80665 * Power(qw, 2) - 9.80665 * Power(qx, 2) -
                                                9.80665 * Power(qy, 2) + 9.80665 * Power(qz, 2)));

        const auto H = List(
            List(-19.6133 * qy, 19.6133 * qz, -19.6133 * qw, 19.6133 * qx, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0),
            List(19.6133 * qx,  19.6133 * qw,  19.6133 * qz, 19.6133 * qy, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0),
            List(19.6133 * qw, -19.6133 * qx, -19.6133 * qy, 19.6133 * qz, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1));

        const Vector3 y = accel - h;

        performMeasurementUpdate(timestamp, y, cov, H);
    }

    void performGnssAccelUpdate(Scalar timestamp, const Vector3& accel, const Matrix3& cov)
    {
        enforce("GNSS accel R", cov.norm() > 0);

        const Scalar qw = x_[0];
        const Scalar qx = x_[1];
        const Scalar qy = x_[2];
        const Scalar qz = x_[3];

        const Scalar alx = x_[7];
        const Scalar aly = x_[8];
        const Scalar alz = x_[9];

        using namespace mathematica;

        const Vector3 h = List(
            List(
                qy * (alz * qw + aly * qx - alx * qy) - qz * (aly * qw - alz * qx + alx * qz) + qw
                    * (alx * qw + alz * qy - aly * qz)
                - qx * (-(alx * qx) - aly * qy - alz * qz)),
            List(
                -(qx * (alz * qw + aly * qx - alx * qy)) + qw * (aly * qw - alz * qx + alx * qz)
                + qz * (alx * qw + alz * qy - aly * qz)
                - qy * (-(alx * qx) - aly * qy - alz * qz)),
            List(
                qw * (alz * qw + aly * qx - alx * qy) + qx * (aly * qw - alz * qx + alx * qz) - qy
                    * (alx * qw + alz * qy - aly * qz)
                - qz * (-(alx * qx) - aly * qy - alz * qz)));

        const auto H = List(
            List(2 * alx * qw + 2 * alz * qy - 2 * aly * qz, 2 * alx * qx + 2 * aly * qy + 2 * alz * qz,
                 2 * alz * qw + 2 * aly * qx - 2 * alx * qy, -2 * aly * qw + 2 * alz * qx - 2 * alx * qz, 0, 0, 0,
                 Power(qw, 2) + Power(qx, 2) - Power(qy, 2) - Power(qz, 2), 2 * qx * qy - 2 * qw * qz,
                 2 * qw * qy + 2 * qx * qz, 0, 0, 0, 0, 0, 0),
            List(2 * aly * qw - 2 * alz * qx + 2 * alx * qz, -2 * alz * qw - 2 * aly * qx + 2 * alx * qy,
                 2 * alx * qx + 2 * aly * qy + 2 * alz * qz, 2 * alx * qw + 2 * alz * qy - 2 * aly * qz, 0, 0, 0,
                 2 * qx * qy + 2 * qw * qz, Power(qw, 2) - Power(qx, 2) + Power(qy, 2) - Power(qz, 2),
                 -2 * qw * qx + 2 * qy * qz, 0, 0, 0, 0, 0, 0),
            List(2 * alz * qw + 2 * aly * qx - 2 * alx * qy, 2 * aly * qw - 2 * alz * qx + 2 * alx * qz,
                 -2 * alx * qw - 2 * alz * qy + 2 * aly * qz, 2 * alx * qx + 2 * aly * qy + 2 * alz * qz, 0, 0, 0,
                 -2 * qw * qy + 2 * qx * qz, 2 * qw * qx + 2 * qy * qz,
                 Power(qw, 2) - Power(qx, 2) - Power(qy, 2) + Power(qz, 2), 0, 0, 0, 0, 0, 0));

        const Vector3 y = accel - h;

        performMeasurementUpdate(timestamp, y, cov, H);
    }

    void performGyroUpdate(Scalar timestamp, const Vector3& angvel, const Matrix3& cov)
    {
        enforce("Gyro R", cov.norm() > 0);

        const Scalar wlx = x_[4];
        const Scalar wly = x_[5];
        const Scalar wlz = x_[6];

        const Scalar bwx = x_[10];
        const Scalar bwy = x_[11];
        const Scalar bwz = x_[12];

        using namespace mathematica;

        const Vector3 h = List(List(bwx + wlx),
                               List(bwy + wly),
                               List(bwz + wlz));

        const auto H = List(List(0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0),
                            List(0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0),
                            List(0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0));

        const Vector3 y = angvel - h;

        performMeasurementUpdate(timestamp, y, cov, H);
    }

    Scalar getTimestamp() const { return state_timestamp_; }

    std::pair<Quaternion, Matrix3> getOutputOrientation() const
    {
        const auto q = getQuat();
        const Scalar qw = q.w();
        const Scalar qx = q.x();
        const Scalar qy = q.y();
        const Scalar qz = q.z();

        /*
         * Quaternion covariance to Euler covariance conversion.
         * Refer to the Mathematica script for derivations.
         * Ref. "Development of a Real-Time Attitude System Using a Quaternion
         * Parameterization and Non-Dedicated GPS Receivers" - John B. Schleppe (page 69)
         */
        using namespace mathematica;
        const Matrix<3, 4> G = List(
            List(
                (2 * qx * (1 - 2 * (Power(qx, 2) + Power(qy, 2)))) / (Power(1 - 2 * (Power(qx, 2) + Power(qy, 2)), 2)
                    + 4 * Power(qw * qx + qy * qz, 2)),
                (2 * qw * (1 - 2 * (Power(qx, 2) + Power(qy, 2)))) / (Power(1 - 2 * (Power(qx, 2) + Power(qy, 2)), 2)
                    + 4 * Power(qw * qx + qy * qz, 2))
                + (8 * qx * (qw * qx + qy * qz)) / (Power(1 - 2 * (Power(qx, 2) + Power(qy, 2)), 2)
                    + 4 * Power(qw * qx + qy * qz, 2)),
                (2 * (1 - 2 * (Power(qx, 2) + Power(qy, 2))) * qz) / (Power(1 - 2 * (Power(qx, 2) + Power(qy, 2)), 2)
                    + 4 * Power(qw * qx + qy * qz, 2))
                + (8 * qy * (qw * qx + qy * qz)) / (Power(1 - 2 * (Power(qx, 2) + Power(qy, 2)), 2)
                    + 4 * Power(qw * qx + qy * qz, 2)),
                (2 * qy * (1 - 2 * (Power(qx, 2) + Power(qy, 2)))) / (Power(1 - 2 * (Power(qx, 2) + Power(qy, 2)), 2)
                    + 4 * Power(qw * qx + qy * qz, 2))),
            List((2 * qy) / Sqrt(1 - 4 * Power(qw * qy - qx * qz, 2)),
                 (-2 * qz) / Sqrt(1 - 4 * Power(qw * qy - qx * qz, 2)),
                 (2 * qw) / Sqrt(1 - 4 * Power(qw * qy - qx * qz, 2)),
                 (-2 * qx) / Sqrt(1 - 4 * Power(qw * qy - qx * qz, 2))),
            List(
                (2 * qz * (1 - 2 * (Power(qy, 2) + Power(qz, 2)))) / (4 * Power(qx * qy + qw * qz, 2)
                    + Power(1 - 2 * (Power(qy, 2) + Power(qz, 2)), 2)),
                (2 * qy * (1 - 2 * (Power(qy, 2) + Power(qz, 2)))) / (4 * Power(qx * qy + qw * qz, 2)
                    + Power(1 - 2 * (Power(qy, 2) + Power(qz, 2)), 2)),
                (8 * qy * (qx * qy + qw * qz)) / (4 * Power(qx * qy + qw * qz, 2)
                    + Power(1 - 2 * (Power(qy, 2) + Power(qz, 2)), 2))
                + (2 * qx * (1 - 2 * (Power(qy, 2) + Power(qz, 2)))) / (4 * Power(qx * qy + qw * qz, 2)
                    + Power(1 - 2 * (Power(qy, 2) + Power(qz, 2)), 2)),
                (8 * qz * (qx * qy + qw * qz)) / (4 * Power(qx * qy + qw * qz, 2)
                    + Power(1 - 2 * (Power(qy, 2) + Power(qz, 2)), 2))
                + (2 * qw * (1 - 2 * (Power(qy, 2) + Power(qz, 2)))) / (4 * Power(qx * qy + qw * qz, 2)
                    + Power(1 - 2 * (Power(qy, 2) + Power(qz, 2)), 2))));

        const Matrix4 C = P_.block<4, 4>(0, 0);
        const Matrix3 cov = G * C * G.transpose();

        return {q, cov};
    }

    std::pair<Vector3, Matrix3> getOutputAngularVelocity() const
    {
        return { getBodyAngVel(), P_.block<3, 3>(4, 4) };
    }

    /// Gravity compensated
    std::pair<Vector3, Matrix3> getOutputAcceleration() const
    {
        return { getBodyAccel(), P_.block<3, 3>(7, 7) };
    }
};

}