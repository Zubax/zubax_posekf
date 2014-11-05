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

#define ZUBAX_POSEKF_STRINGIZE2(x)      #x
#define ZUBAX_POSEKF_STRINGIZE(x)       ZUBAX_POSEKF_STRINGIZE2(x)
#define ZUBAX_POSEKF_MAKE_ERROR_MSG(x)  (__FILE__ ":" ZUBAX_POSEKF_STRINGIZE(__LINE__) ": " #x)
#define ZUBAX_POSEKF_ENFORCE(x)                                                 \
    if (!(x)) {                                                                 \
        ROS_ERROR(ZUBAX_POSEKF_MAKE_ERROR_MSG(x));                              \
        throw ::zubax_posekf::FilterException(ZUBAX_POSEKF_MAKE_ERROR_MSG(x));  \
    }

namespace zubax_posekf
{

using Scalar = double;
using Quaternion = Eigen::Quaternion<Scalar>;
template <int Rows, int Cols> using Matrix = Eigen::Matrix<Scalar, Rows, Cols>;
template <int Size> using Vector = Matrix<Size, 1>;
using Vector3 = Vector<3>;
using Matrix3 = Matrix<3, 3>;
using Matrix4 = Matrix<4, 4>;

class FilterException : std::runtime_error
{
public:
    FilterException(const std::string& text) : std::runtime_error(text) { }
};

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

template <typename Value, typename Min, typename Max>
inline bool checkRangeAndConstrain(Value& inout_value, const Min& min, const Max& max)
{
    ROS_ASSERT(min < max);
    if (inout_value < min)
    {
        inout_value = min;
        return false;
    }
    if (inout_value > max)
    {
        inout_value = max;
        return false;
    }
    return true;
}

template <typename Value, typename Limit>
inline bool checkRangeAndConstrainSymmetric(Value& inout_value, const Limit& limit)
{
    const auto abs_limit = std::abs(limit);
    return checkRangeAndConstrain(inout_value, -abs_limit, abs_limit);
}

template <typename Scalar, int Size, int Options>
inline bool validateAndFixCovarianceMatrix(Eigen::Matrix<Scalar, Size, Size, Options>& matrix,
                                           const Scalar& covariance_abs_limit, const Scalar& min_variance)
{
    bool retval = true;
    Scalar* const ptr = matrix.data();

    ROS_ASSERT(ptr != nullptr);
    ROS_ASSERT(covariance_abs_limit > 0);
    ROS_ASSERT(min_variance > 0);

    for (int i = 0; i < (Size * Size); i++)
    {
        const bool on_diagonal = (i / Size) == (i % Size);
        const Scalar min = on_diagonal ? min_variance : -covariance_abs_limit;
        retval = checkRangeAndConstrain(ptr[i], min, covariance_abs_limit) ? retval : false;
    }

    matrix = 0.5 * (matrix + matrix.transpose());  // Make sure the matrix stays symmetric
    return retval;
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

    const Scalar AccelCovMult = 1e3;
    const Scalar MaxGyroDrift = 0.1;
    const Scalar MaxCovariance = 1e6;

    Scalar state_timestamp_ = 0.0;
    bool initialized_ = false;

    Vector3 accel_;       ///< This is not included in the state vector
    Matrix3 accel_cov_;   ///< Ditto

    Quaternion getQuat() const  { return Quaternion(x_[0], x_[1], x_[2], x_[3]); }
    Vector3 getAngVel() const   { return Vector3(x_[4], x_[5], x_[6]); }
    Vector3 getGyroBias() const { return Vector3(x_[7], x_[8], x_[9]); }

    void normalizeAndCheck()
    {
        // Quaternion normalization
        const auto q = getQuat().normalized();
        x_[0] = q.w();
        x_[1] = q.x();
        x_[2] = q.y();
        x_[3] = q.z();

        // Gyro drift constrain
        for (int i = 7; i <= 9; i++)
        {
            if (!checkRangeAndConstrainSymmetric(x_[i], MaxGyroDrift))
            {
                ROS_WARN_THROTTLE(1, "Gyro drift is too high");
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

        ZUBAX_POSEKF_ENFORCE(std::isfinite(x_.sum()));
        ZUBAX_POSEKF_ENFORCE(std::isfinite(P_.sum()));
        ZUBAX_POSEKF_ENFORCE(std::isfinite(Q_.sum()));

        ZUBAX_POSEKF_ENFORCE(std::isfinite(state_timestamp_) && (state_timestamp_ > 0.0));
    }

    template <int NumStates>
    void performMeasurementUpdate(Scalar timestamp,
                                  const Vector<NumStates>& y,
                                  const Matrix<NumStates, NumStates>& R,
                                  const Matrix<NumStates, 10>& H)
    {
        ROS_ASSERT(initialized_);
        ROS_ASSERT(timestamp > 0);

        state_timestamp_ = timestamp;

        const Matrix<NumStates, NumStates> R_sym = 0.5 * (R + R.transpose());  // Ensure that R is symmetric

        Matrix<NumStates, NumStates> S_inv;
        {
            const Matrix<NumStates, NumStates> S = H * P_ * H.transpose() + R_sym;
            bool s_is_invertible = false;
            S.computeInverseWithCheck(S_inv, s_is_invertible);
            ZUBAX_POSEKF_ENFORCE(s_is_invertible);
        }

        const auto K = static_cast<Matrix<10, NumStates> >(P_ * H.transpose() * S_inv);

        x_ = x_ + K * y;

        // Joseph form is used instead of the simplified form to improve numerical stability
        const Matrix<10, 10> IKH = decltype(P_)::Identity() - K * H;
        P_ = IKH * P_ * IKH.transpose() + K * R_sym * K.transpose();

        normalizeAndCheck();
    }

public:
    IMUFilter()
    {
        x_.setZero();
        P_.setZero();

        // TODO: runtime Q estimation
        const Scalar QVarQuat     = 1e-6;
        const Scalar QVarAngVel   = 1e+0;
        const Scalar QVarGyroBias = 1e-6;

        Vector<decltype(Q_)::RowsAtCompileTime> Q_diag;
        Q_diag.setZero();
        Q_diag <<
            QVarQuat, QVarQuat, QVarQuat, QVarQuat,
            QVarAngVel, QVarAngVel, QVarAngVel,
            QVarGyroBias, QVarGyroBias, QVarGyroBias;
        Q_ = Q_diag.asDiagonal();

        accel_.setZero();
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

        P_ = decltype(P_)::Identity() * 100.0;

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
        const Quaternion delta_quat = quaternionFromEuler(getAngVel() * dtf);
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
        const Matrix<10, 10> F = List(
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

        P_ = F * P_ * F.transpose() + Q_;

        normalizeAndCheck();
    }

    void performAccelUpdate(Scalar timestamp, const Vector3& accel, const Matrix3& cov)
    {
        ZUBAX_POSEKF_ENFORCE(cov.norm() > 0);

        accel_ = accel;
        accel_cov_ = cov;

        const Matrix3 R = cov * AccelCovMult;

        const Scalar qw = x_[0];
        const Scalar qx = x_[1];
        const Scalar qy = x_[2];
        const Scalar qz = x_[3];

        using namespace mathematica;

        const Vector3 h = List(List(-2 * qw * qy + 2 * qx * qz),
                               List(2 * qw * qx + 2 * qy * qz),
                               List(Power(qw, 2) - Power(qx, 2) - Power(qy, 2) + Power(qz, 2)));

        const Matrix<3, 10> H = List(List(-2 * qy, 2 * qz, -2 * qw, 2 * qx, 0, 0, 0, 0, 0, 0),
                                     List(2 * qx, 2 * qw, 2 * qz, 2 * qy, 0, 0, 0, 0, 0, 0),
                                     List(2 * qw, -2 * qx, -2 * qy, 2 * qz, 0, 0, 0, 0, 0, 0));

        const Vector3 y = accel.normalized() - h;

        performMeasurementUpdate(timestamp, y, R, H);
    }

    void performGyroUpdate(Scalar timestamp, const Vector3& angvel, const Matrix3& cov)
    {
        ZUBAX_POSEKF_ENFORCE(cov.norm() > 0);

        const Scalar wlx = x_[4];
        const Scalar wly = x_[5];
        const Scalar wlz = x_[6];

        const Scalar bwlx = x_[7];
        const Scalar bwly = x_[8];
        const Scalar bwlz = x_[9];

        using namespace mathematica;

        const Vector3 h = List(List(bwlx + wlx),
                               List(bwly + wly),
                               List(bwlz + wlz));

        const Matrix<3, 10> H = List(List(0, 0, 0, 0, 1, 0, 0, 1, 0, 0),
                                     List(0, 0, 0, 0, 0, 1, 0, 0, 1, 0),
                                     List(0, 0, 0, 0, 0, 0, 1, 0, 0, 1));

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
        return { x_.block<3, 1>(4, 0), P_.block<3, 3>(4, 4) };
    }

    std::pair<Vector3, Matrix3> getOutputAcceleration() const
    {
        return { accel_, accel_cov_ };
    }
};

}
