/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Please refer to the file LICENSE for terms and conditions.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#pragma once

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <cmath>

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

}
