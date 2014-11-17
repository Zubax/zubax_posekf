/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Please refer to the file LICENSE for terms and conditions.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_ros/transform_broadcaster.h>
#include "gnss_provider.hpp"
#include "imu_provider.hpp"
#include "filter.hpp"
#include "linear_algebra.hpp"
#include "exception.hpp"

namespace zubax_posekf
{
/**
 * ROS adapter for the filter class
 */
class FilterWrapper
{
    mutable ros::Publisher pub_imu_;
    mutable tf2_ros::TransformBroadcaster pub_tf_;

    Filter filter_;
    GNSSProvider gnss_provider_;
    IMUProvider imu_provider_;

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
        tf.header.frame_id = "base_link";
        tf.header.stamp = msg.header.stamp;
        tf.child_frame_id = msg.header.frame_id;
        tf.transform.rotation = msg.orientation;
        tf.transform.translation.x = 0.5;            // Add an offset to improve visualization
        pub_tf_.sendTransform(tf);
    }

    void cbImu(const IMUSample& sample)
    {
        if (filter_.getTimestamp() >= sample.timestamp.toSec())
        {
            ROS_WARN_THROTTLE(1, "IMU update from the past [%f sec]",
                              filter_.getTimestamp() - sample.timestamp.toSec());
            return;
        }

        /*
         * Publishing the original IMU transform
         */
        geometry_msgs::TransformStamped tf;
        tf.header.frame_id = "base_link";
        tf.header.stamp = sample.timestamp;
        tf.child_frame_id = "zubax_posekf_in";
        tf::quaternionEigenToMsg(sample.orientation, tf.transform.rotation);
        pub_tf_.sendTransform(tf);

        /*
         * Filter update
         * TODO: proper initialization
         */
        if (!filter_.isInitialized())
        {
            filter_.initialize(sample.timestamp.toSec(),
                               quaternionFromEuler(Vector3(0, 0, -M_PI / 2.0)) * sample.orientation);
            return;
        }

        filter_.performTimeUpdate(sample.timestamp.toSec());
        filter_.performAccelUpdate(sample.accel, sample.accel_covariance);
        filter_.performGyroUpdate(sample.gyro, sample.gyro_covariance);

        publishEstimations();
    }

    void cbGnss(const GNSSLocalPosVel& local, const gps_common::GPSFix&)
    {
        /*
         * Publishing the location transform
         */
        geometry_msgs::TransformStamped tf;
        tf.header.frame_id = "world";
        tf.header.stamp = local.timestamp + ros::Duration().fromSec(0.1);
        tf.child_frame_id = "base_link";
        tf.transform.rotation.w = 1.0;
        tf::vectorEigenToMsg(local.position, tf.transform.translation);
        pub_tf_.sendTransform(tf);

        if (filter_.getTimestamp() >= (local.timestamp.toSec() + 0.3))  // TODO: get rid of this later
        {
            ROS_WARN_THROTTLE(1, "GNSS update from the past [%f sec]",
                              filter_.getTimestamp() - local.timestamp.toSec());
            return;
        }

        if (!filter_.isInitialized())
        {
            ROS_WARN_THROTTLE(1, "GNSS update skipped - not inited yet");
            return;
        }

        filter_.performTimeUpdate(local.timestamp.toSec());
        filter_.performGNSSPosUpdate(local.position, local.position_covariance);
        filter_.performGNSSVelUpdate(local.velocity, local.velocity_covariance);

        publishEstimations();
    }

public:
    FilterWrapper(ros::NodeHandle& node)
        : gnss_provider_(node, "gnss")
        , imu_provider_(node, "imu")
    {
        gnss_provider_.on_sample = std::bind(&FilterWrapper::cbGnss, this,
                                             std::placeholders::_1, std::placeholders::_2);

        imu_provider_.on_sample = std::bind(&FilterWrapper::cbImu, this, std::placeholders::_1);

        pub_imu_ = node.advertise<sensor_msgs::Imu>("out", 10);

        ROS_INFO("FilterWrapper inited");
    }
};

}

int main(int argc, char** argv)
{
    try
    {
        ros::init(argc, argv, "posekf");

        ros::NodeHandle node;

        zubax_posekf::FilterWrapper wrapper(node);

        ros::spin();
    }
    catch (const zubax_posekf::Exception& ex)
    {
        std::cerr << "zubax_posekf::Exception: " << ex.what() << std::endl;
        return -1;
    }

    return 0;
}
