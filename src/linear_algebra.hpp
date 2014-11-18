/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Please refer to the file LICENSE for terms and conditions.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#pragma once

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <cmath>
#include "mathematica.hpp"

namespace zubax_posekf
{

using Scalar = double;
using Quaternion = Eigen::Quaternion<Scalar>;
template <int Rows, int Cols> using Matrix = Eigen::Matrix<Scalar, Rows, Cols>;
template <int Size> using Vector = Matrix<Size, 1>;
using Vector3 = Vector<3>;
using Matrix3 = Matrix<3, 3>;
using Matrix4 = Matrix<4, 4>;

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

/**
 * Quaternion covariance to Euler covariance conversion jacobian.
 * Refer to the Mathematica script for derivations.
 * Ref. "Development of a Real-Time Attitude System Using a Quaternion
 * Parameterization and Non-Dedicated GPS Receivers" - John B. Schleppe (page 69)
 */
inline Matrix<3, 4> quaternionToEulerJacobian(const Quaternion& q)
{
    const Scalar qw = q.w();
    const Scalar qx = q.x();
    const Scalar qy = q.y();
    const Scalar qz = q.z();

    using namespace mathematica;

    const auto tmpcse0 = Power(qz, 2);
    const auto tmpcse1 = Power(qy, 2);
    const auto tmpcse2 = Power(qw, 2);
    const auto tmpcse3 = Power(qx, 2);
    const auto tmpcse4 = qw * qy;
    const auto tmpcse5 = 4 * Power(qy, 4);
    const auto tmpcse6 = -4 * tmpcse1;
    const auto tmpcse7 = 4 * qx * qy * qz;
    const auto tmpcse8 = 2 * tmpcse0;
    const auto tmpcse9 = 2 * tmpcse1;
    const auto tmpcse10 = 2 * tmpcse3;
    const auto tmpcse11 = 8 * qx * qz * tmpcse4;
    const auto tmpcse12 = -2 * qw * tmpcse1;
    const auto tmpcse13 = -1 + tmpcse8 + tmpcse9;
    const auto tmpcse14 = -1 + tmpcse10 + tmpcse9;
    const auto tmpcse15 = 1 / Sqrt(1 - 4 * Power(-(qx * qz) + tmpcse4, 2));
    const auto tmpcse16 = 1
        / (1 + 4 * Power(qz, 4) - 4 * tmpcse0 + 8 * tmpcse0 * tmpcse1 + tmpcse11 + 4 * tmpcse0 * tmpcse2
           + 4 * tmpcse1 * tmpcse3 + tmpcse5 + tmpcse6);
    const auto tmpcse17 = 1
        / (1 + 4 * Power(qx, 4) + 4 * tmpcse0 * tmpcse1 + tmpcse11 - 4 * tmpcse3 + 8 * tmpcse1 * tmpcse3
           + 4 * tmpcse2 * tmpcse3 + tmpcse5 + tmpcse6);
    const auto tmp = List(
        List(-2 * qx * tmpcse14 * tmpcse17, 2 * tmpcse17 * (qw + qw * tmpcse10 + tmpcse12 + tmpcse7),
             2 * tmpcse17 * (qz - 2 * qz * tmpcse3 + 4 * qx * tmpcse4 + qz * tmpcse9), -2 * qy * tmpcse14 * tmpcse17),
        List(2 * qy * tmpcse15, -2 * qz * tmpcse15, 2 * qw * tmpcse15, -2 * qx * tmpcse15),
        List(-2 * qz * tmpcse13 * tmpcse16, -2 * qy * tmpcse13 * tmpcse16,
             2 * tmpcse16 * (qx - 2 * qx * tmpcse0 + 4 * qz * tmpcse4 + qx * tmpcse9),
             2 * tmpcse16 * (qw + tmpcse12 + tmpcse7 + qw * tmpcse8)));
    return tmp;
}

/**
 * Euler covariance to Quaternion covariance conversion jacobian.
 * Refer to the Mathematica script for derivations.
 * Ref. "Development of a Real-Time Attitude System Using a Quaternion
 * Parameterization and Non-Dedicated GPS Receivers" - John B. Schleppe (page 69)
 */
inline Matrix<4, 3> quaternionFromEulerJacobian(const Vector3& roll_pitch_yaw)
{
    const Scalar roll = roll_pitch_yaw[0];
    const Scalar pitch = roll_pitch_yaw[1];
    const Scalar yaw = roll_pitch_yaw[2];

    using namespace mathematica;

    const auto tmpcse0 = yaw / 2.;
    const auto tmpcse1 = roll / 2.;
    const auto tmpcse2 = pitch / 2.;
    const auto tmpcse3 = Sin(tmpcse0);
    const auto tmpcse4 = Sin(tmpcse1);
    const auto tmpcse5 = Sin(tmpcse2);
    const auto tmpcse6 = Cos(tmpcse0);
    const auto tmpcse7 = Cos(tmpcse1);
    const auto tmpcse8 = Cos(tmpcse2);
    const auto tmpcse9 = tmpcse3 * tmpcse4 * tmpcse5;
    const auto tmpcse10 = tmpcse6 * tmpcse7 * tmpcse8;
    const auto tmpcse11 = -(tmpcse3 * tmpcse5 * tmpcse7);
    const auto tmpcse12 = -(tmpcse4 * tmpcse6 * tmpcse8);
    const auto tmpcse13 = -(tmpcse5 * tmpcse6 * tmpcse7);
    const auto tmpcse14 = -(tmpcse4 * tmpcse5 * tmpcse6);
    const auto tmpcse15 = -(tmpcse3 * tmpcse7 * tmpcse8);
    const auto tmpcse16 = (tmpcse10 + tmpcse9) / 2.;
    const auto tmpcse17 = (tmpcse13 - tmpcse3 * tmpcse4 * tmpcse8) / 2.;
    const auto tmp = List(
        List((tmpcse12 + tmpcse3 * tmpcse5 * tmpcse7) / 2., (tmpcse13 + tmpcse3 * tmpcse4 * tmpcse8) / 2.,
             (tmpcse15 + tmpcse4 * tmpcse5 * tmpcse6) / 2.),
        List(tmpcse16, (tmpcse14 + tmpcse15) / 2., tmpcse17),
        List((tmpcse14 + tmpcse3 * tmpcse7 * tmpcse8) / 2., (tmpcse10 - tmpcse9) / 2.,
             (tmpcse11 + tmpcse4 * tmpcse6 * tmpcse8) / 2.),
        List(tmpcse17, (tmpcse11 + tmpcse12) / 2., tmpcse16));
    return tmp;
}

/**
 * Rotates a vector by quaternion.
 * Quaternions - Madgwick, 2011 - eq. 5
 * Autogenerated; see the Mathematica scripts for details.
 */
inline Vector3 rotateVectorByQuaternion(const Vector3& vec, const Quaternion& quat)
{
    const Scalar vx = vec.x();
    const Scalar vy = vec.y();
    const Scalar vz = vec.z();

    const Scalar qw = quat.w();
    const Scalar qx = quat.x();
    const Scalar qy = quat.y();
    const Scalar qz = quat.z();

    using namespace mathematica;
    return List(
        List(
            qy * (-(qy * vx) + qx * vy + qw * vz) - qz * (qz * vx + qw * vy - qx * vz) + qw
                * (qw * vx - qz * vy + qy * vz)
            - qx * (-(qx * vx) - qy * vy - qz * vz)),
        List(
            -(qx * (-(qy * vx) + qx * vy + qw * vz)) + qw * (qz * vx + qw * vy - qx * vz)
            + qz * (qw * vx - qz * vy + qy * vz)
            - qy * (-(qx * vx) - qy * vy - qz * vz)),
        List(
            qw * (-(qy * vx) + qx * vy + qw * vz) + qx * (qz * vx + qw * vy - qx * vz) - qy
                * (qw * vx - qz * vy + qy * vz)
            - qz * (-(qx * vx) - qy * vy - qz * vz)));
}

/**
 * Covariance matrix rotation Jacobian.
 * jacobian[rotateVectorByQuaternion[{{vx}, {vy}, {vz}}, Quaternion[qw, qx, qy, qz]], {vx, vy, vz}]
 * Autogenerated; see the Mathematica scripts for details.
 */
inline Matrix3 rotateVectorByQuaternionJacobian(const Quaternion& quat)
{
    const Scalar qw = quat.w();
    const Scalar qx = quat.x();
    const Scalar qy = quat.y();
    const Scalar qz = quat.z();

    using namespace mathematica;
    return List(
        List(Power(qw, 2) + Power(qx, 2) - Power(qy, 2) - Power(qz, 2), 2 * qx * qy - 2 * qw * qz,
             2 * qw * qy + 2 * qx * qz),
        List(2 * qx * qy + 2 * qw * qz, Power(qw, 2) - Power(qx, 2) + Power(qy, 2) - Power(qz, 2),
             -2 * qw * qx + 2 * qy * qz),
        List(-2 * qw * qy + 2 * qx * qz, 2 * qw * qx + 2 * qy * qz,
             Power(qw, 2) - Power(qx, 2) - Power(qy, 2) + Power(qz, 2)));
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

/**
 * @param array Row-major flat array
 * @return      Matrix in default representation (column-major)
 */
template <int Rows, int Cols, typename Scalar = double>
inline Eigen::Matrix<Scalar, Rows, Cols> matrixMsgToEigen(boost::array<Scalar, Rows * Cols> array)
{
    return Eigen::Map<Eigen::Matrix<Scalar, Rows, Cols, Eigen::RowMajor>>(array.data());
}

/**
 * @param matrix Any matrix
 * @return       Row-major flat array
 */
template <int Rows, int Cols, typename Scalar, int Options>
inline boost::array<Scalar, Rows * Cols> matrixEigenToMsg(const Eigen::Matrix<Scalar, Rows, Cols, Options>& matrix)
{
    const Eigen::Matrix<Scalar, Rows, Cols, Eigen::RowMajor> row_major(matrix);
    boost::array<Scalar, Rows * Cols> array;
    std::copy_n(row_major.data(), Rows * Cols, array.begin());
    return array;
}

}
