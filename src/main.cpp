/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Please refer to the file LICENSE for terms and conditions.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <gps_common/GPSFix.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "filter.hpp"
#include "linear_algebra.hpp"
#include "exception.hpp"

namespace zubax_posekf
{
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

/**
 * ROS adapter for the filter class
 */
class IMUFilterWrapper
{
    IMUFilter filter_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_gnss_;

    Scalar prev_gnss_update_ = 0.0;
    Vector3 prev_gnss_vel_;

    mutable DebugPublisher pub_debug_;
    mutable ros::Publisher pub_imu_;
    mutable tf2_ros::TransformBroadcaster pub_tf_;

    void publishEstimations() const
    {
        sensor_msgs::Imu msg;
        msg.header.frame_id = "zubax_posekf_out";
        msg.header.stamp.fromSec(filter_.getTimestamp());

        {
            const auto orientation = filter_.getOutputOrientation();
            tf::quaternionEigenToMsg(orientation.first, msg.orientation);
            msg.orientation_covariance = matrixEigenToMsg(orientation.second);
        }
        {
            const auto accel = filter_.getOutputAcceleration();
            tf::vectorEigenToMsg(accel.first, msg.linear_acceleration);
            msg.linear_acceleration_covariance = matrixEigenToMsg(accel.second);
        }
        {
            const auto angvel = filter_.getOutputAngularVelocity();
            tf::vectorEigenToMsg(angvel.first, msg.angular_velocity);
            msg.angular_velocity_covariance = matrixEigenToMsg(angvel.second);
        }

        pub_imu_.publish(msg);

        /*
         * Publishing the second IMU transform
         */
        geometry_msgs::TransformStamped tf;
        tf.header.frame_id = "map";
        tf.header.stamp = msg.header.stamp;
        tf.child_frame_id = msg.header.frame_id;
        tf.transform.rotation = msg.orientation;
        tf.transform.translation.x = 0.5;            // Add an offset to improve visualization
        pub_tf_.sendTransform(tf);
    }

    void cbImu(const sensor_msgs::Imu& msg)
    {
        const double timestamp = msg.header.stamp.toSec();
        if (filter_.getTimestamp() >= timestamp)
        {
            ROS_WARN_THROTTLE(1, "IMU update from the past [%f sec]", filter_.getTimestamp() - timestamp);
            return;
        }

        /*
         * Publishing the original IMU transform
         */
        geometry_msgs::TransformStamped tf;
        tf.header.frame_id = "map";
        tf.header.stamp = msg.header.stamp;
        tf.child_frame_id = "zubax_posekf_in";
        tf.transform.rotation = msg.orientation;
        pub_tf_.sendTransform(tf);

        /*
         * Msg --> Eigen conversion
         */
        Quaternion quat;
        tf::quaternionMsgToEigen(msg.orientation, quat);
        Vector3 accel;
        tf::vectorMsgToEigen(msg.linear_acceleration, accel);
        Vector3 angvel;
        tf::vectorMsgToEigen(msg.angular_velocity, angvel);

        const auto orientation_cov = matrixMsgToEigen<3, 3>(msg.orientation_covariance);
        auto accel_cov  = matrixMsgToEigen<3, 3>(msg.linear_acceleration_covariance);
        auto angvel_cov = matrixMsgToEigen<3, 3>(msg.angular_velocity_covariance);

        if (accel_cov.norm() <= 0)
        {
            accel_cov = decltype(accel_cov)::Identity() * 1.0;
        }

        if (angvel_cov.norm() <= 0)
        {
            angvel_cov = decltype(angvel_cov)::Identity() * 0.01;
        }

        /*
         * Filter update
         */
        if (!filter_.isInitialized())
        {
            filter_.initialize(timestamp, quat, orientation_cov);
            return;
        }

        filter_.performTimeUpdate(timestamp);
        filter_.performAccelUpdate(timestamp, accel, accel_cov);
        filter_.performGyroUpdate(timestamp, angvel, angvel_cov);

        publishEstimations();
    }

    void cbGnss(const gps_common::GPSFix& msg)
    {
        const double timestamp = msg.header.stamp.toSec();
        if (filter_.getTimestamp() >= timestamp)
        {
            ROS_WARN_THROTTLE(1, "GNSS update from the past [%f sec]", filter_.getTimestamp() - timestamp);
            return;
        }

        using namespace mathematica;

        /*
         * Speed vector and covariance - conversion from polar to cartesian
         */
        const Scalar gnssTrack = msg.track * Degree;
        const Scalar gnssSpeed = msg.speed;
        const Scalar gnssClimb = msg.climb;

        enforce("GNSS data", (Abs(gnssTrack) <= 2.0 * Pi) && (Abs(gnssSpeed) < 515) && (Abs(gnssClimb) < 515));

        const auto vel = List(List(gnssSpeed * Sin(gnssTrack)),  // Lon
                              List(gnssSpeed * Cos(gnssTrack)),  // Lat
                              List(gnssClimb));                  // Climb
        Matrix3 Rvel;
        {
            const Scalar err_track_rad = ((msg.err_track > 0) ? msg.err_track : 30.0) * Degree;

            const auto Rpolar = List(List(Power(err_track_rad, 2), 0, 0),  // stdev --> covariance matrix
                                     List(0, Power(msg.err_speed, 2), 0),
                                     List(0, 0, Power(msg.err_climb, 2)));

            enforce("GNSS R polar", (Rpolar(0, 0) > 0) && (Rpolar(1, 1) > 0) && (Rpolar(2, 2) > 0));

            const auto Gpolar = List(List(gnssSpeed * Cos(gnssTrack), Sin(gnssTrack), 0),
                                     List(-(gnssSpeed * Sin(gnssTrack)), Cos(gnssTrack), 0),
                                     List(0, 0, 1));

            Rvel = Gpolar * Rpolar * Gpolar.transpose();

            enforce("GNSS R vel", (Rvel(0, 0) > 0) && (Rvel(1, 1) > 0) && (Rvel(2, 2) > 0));
        }

        /*
         * Acceleration computation
         */
        Vector3 accel;
        Matrix3 Raccel;

        accel.setZero();
        Raccel = Matrix3::Identity() * 1e6;

        if (prev_gnss_update_ > 0.0)
        {
            const Scalar dt = timestamp - prev_gnss_update_;
            enforce("Non-positive dt", dt > 0);

            accel = (vel - prev_gnss_vel_) / dt;

            const Matrix3 G = Matrix3::Identity() / dt;  // {{1/dt, 0, 0}, {0, 1/dt, 0}, {0, 0, 1/dt}}
            Raccel = G * Rvel * G.transpose();
        }
        else
        {
            ROS_INFO("GNSS: First message");
        }
        prev_gnss_update_ = timestamp;
        prev_gnss_vel_ = vel;

        /*
         * Debugging data
         */
        pub_debug_.publish("gnss_vel", vel);
        pub_debug_.publish("gnss_vel_cov", Rvel);
        pub_debug_.publish("gnss_accel", accel);
        pub_debug_.publish("gnss_accel_cov", Raccel);

        /*
         * Filter update
         */
        if (!filter_.isInitialized())
        {
            ROS_WARN_THROTTLE(1, "GNSS update skipped - not inited yet");
            return;
        }

        filter_.performTimeUpdate(timestamp);
        filter_.performGnssAccelUpdate(timestamp, accel, Raccel);
    }

public:
    IMUFilterWrapper(ros::NodeHandle& node, unsigned queue_size = 10)
    {
        sub_imu_  = node.subscribe("imu",  queue_size, &IMUFilterWrapper::cbImu, this);
        sub_gnss_ = node.subscribe("gnss", queue_size, &IMUFilterWrapper::cbGnss, this);

        pub_imu_ = node.advertise<sensor_msgs::Imu>("out", queue_size);

        prev_gnss_vel_.setZero();

        ROS_INFO("IMUFilterWrapper inited");
    }
};

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "posekf");

    ros::NodeHandle node;

    zubax_posekf::IMUFilterWrapper wrapper(node);

    ros::spin();

    return 0;
}
