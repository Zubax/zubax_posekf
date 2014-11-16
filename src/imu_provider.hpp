/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */


#pragma once

#include <functional>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <eigen_conversions/eigen_msg.h>
#include "linear_algebra.hpp"
#include "exception.hpp"

namespace zubax_posekf
{
/**
 * Converted IMU data that can be directly supplied to the filter
 */
struct IMUSample
{
    ros::Time timestamp;

    Vector3 accel;
    Matrix3 accel_covariance;

    Vector3 gyro;
    Matrix3 gyro_covariance;

    Quaternion orientation;

    IMUSample()
    {
        accel.setZero();
        accel_covariance.setZero();

        gyro.setZero();
        gyro_covariance.setZero();

        orientation.setIdentity();
    }

    bool isValid() const { return timestamp.isValid(); }
};

/**
 * IMU data converter
 */
class IMUProvider
{
    ros::Subscriber sub_imu_;

    IMUSample last_sample_;

    struct Config
    {
        double default_accel_variance = 1.0;
        double default_gyro_variance = 0.01;

        Config()
        {
            ros::NodeHandle node("~imu");

            (void)node.getParam("default_accel_variance", default_accel_variance);
            (void)node.getParam("default_gyro_variance", default_gyro_variance);

            ROS_INFO("IMU Provider:\n"
                     "\t~imu/default_accel_variance = %f\n"
                     "\t~imu/default_gyro_variance  = %f",
                     default_accel_variance,
                     default_gyro_variance);
        }
    } const config_;

    void cbImu(const sensor_msgs::Imu& msg)
    {
        last_sample_.timestamp = msg.header.stamp;

        tf::quaternionMsgToEigen(msg.orientation, last_sample_.orientation);
        tf::vectorMsgToEigen(msg.linear_acceleration, last_sample_.accel);
        tf::vectorMsgToEigen(msg.angular_velocity, last_sample_.gyro);

        last_sample_.accel_covariance = matrixMsgToEigen<3, 3>(msg.linear_acceleration_covariance);
        if (last_sample_.accel_covariance.norm() <= 0.0)
        {
            last_sample_.accel_covariance = Matrix3::Identity() * config_.default_accel_variance;
        }

        last_sample_.gyro_covariance  = matrixMsgToEigen<3, 3>(msg.angular_velocity_covariance);
        if (last_sample_.gyro_covariance.norm() <= 0.0)
        {
            last_sample_.gyro_covariance = Matrix3::Identity() * config_.default_gyro_variance;
        }

        if (on_sample)
        {
            on_sample(last_sample_);
        }
    }

public:
    IMUProvider(ros::NodeHandle& node, const std::string topic)
    {
        sub_imu_ = node.subscribe(topic, 10, &IMUProvider::cbImu, this);
    }

    IMUSample getLastSample() const { return last_sample_; }

    std::function<void (const IMUSample&)> on_sample;
};

}
