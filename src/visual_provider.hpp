/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */


#pragma once

#include <functional>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include "linear_algebra.hpp"
#include "debug_publisher.hpp"
#include "exception.hpp"
#include "mathematica.hpp"

namespace zubax_posekf
{
/**
 * Visual input
 */
struct VisualSample
{
    ros::Time timestamp;

    Vector3 position;
    Quaternion orientation;
    Matrix<6, 6> position_orientation_cov;

    VisualSample()
    {
        position.setZero();
        orientation.setIdentity();
        position_orientation_cov.setZero();
    }
};

/**
 * Camera pose data
 */
struct CameraTransform
{
    Vector3 translation;
    Quaternion rotation;

    CameraTransform()
    {
        translation.setZero();
        rotation.setIdentity();
    }
};

/**
 * Visual SLAM or VO input provider
 */
class VisualProvider
{
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Subscriber sub_odom_;
    CameraTransform camera_transform_;
    bool ready_ = false;

    void cbOdom(const nav_msgs::Odometry& msg)
    {
        try
        {
            // TODO: obtain the initial transform from odom to the IMU frame; let the filter refine it
//            geometry_msgs::TransformStamped transform =
//                tf_buffer_.lookupTransform(msg.child_frame_id, msg.header.frame_id, ros::Time(0));
//
//            tf::vectorMsgToEigen(transform.transform.translation, camera_transform_.translation);
//            tf::quaternionMsgToEigen(transform.transform.rotation, camera_transform_.rotation);
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_WARN_THROTTLE(1, "VisualProvider: Skip due to transform lookup failure: %s", ex.what());
            return;
        }

        if (msg.pose.covariance[0] <= 0)
        {
            ROS_WARN_THROTTLE(1, "VisualProvider: Skip due to invalid covariance");
            return;
        }

        VisualSample sample;
        sample.timestamp = msg.header.stamp;
        sample.position_orientation_cov = matrixMsgToEigen<6, 6>(msg.pose.covariance);

        tf::pointMsgToEigen(msg.pose.pose.position, sample.position);
        tf::quaternionMsgToEigen(msg.pose.pose.orientation, sample.orientation);

        if (on_sample)
        {
            on_sample(sample, camera_transform_);
        }
    }

public:
    VisualProvider(ros::NodeHandle& node, const std::string odom_topic)
        : tf_listener_(tf_buffer_)
    {
        sub_odom_ = node.subscribe(odom_topic, 10, &VisualProvider::cbOdom, this);
    }

    bool isReady() const { return ready_; }

    CameraTransform getCameraTransform() const { return camera_transform_; }

    std::function<void (const VisualSample&, const CameraTransform&)> on_sample;
};

}
