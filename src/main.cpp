/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Please refer to the file LICENSE for terms and conditions.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <eigen_conversions/eigen_msg.h>
#include "imu_filter.hpp"

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
    mutable ros::Publisher pub_imu_;

    void publish(const std::string& frame_id) const
    {
        sensor_msgs::Imu msg;
        msg.header.frame_id = frame_id;
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
    }

    void cbImu(const sensor_msgs::Imu& msg)
    {
        const double timestamp = msg.header.stamp.toSec();

        /*
         * Msg --> Eigen conversion
         */
        Eigen::Quaterniond quat;
        tf::quaternionMsgToEigen(msg.orientation, quat);
        Eigen::Vector3d accel;
        tf::vectorMsgToEigen(msg.linear_acceleration, accel);
        Eigen::Vector3d angvel;
        tf::vectorMsgToEigen(msg.angular_velocity, angvel);

        const auto orientation_cov = matrixMsgToEigen<3, 3>(msg.orientation_covariance);
        const auto accel_cov       = matrixMsgToEigen<3, 3>(msg.linear_acceleration_covariance);
        const auto angvel_cov      = matrixMsgToEigen<3, 3>(msg.angular_velocity_covariance);

        /*
         * Filter update
         */
        if (!filter_.isInitialized())
        {
            filter_.initialize(timestamp, quat, orientation_cov);
            return;
        }

        filter_.performAccelUpdate(timestamp, accel, accel_cov);
        filter_.performGyroUpdate(timestamp, angvel, angvel_cov);

        publish(msg.header.frame_id);
    }

public:
    IMUFilterWrapper(ros::NodeHandle& node, unsigned queue_size = 10)
    {
        sub_imu_ = node.subscribe("in", queue_size, &IMUFilterWrapper::cbImu, this);
        pub_imu_ = node.advertise<sensor_msgs::Imu>("out", queue_size);
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
