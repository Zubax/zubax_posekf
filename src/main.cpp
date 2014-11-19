/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Please refer to the file LICENSE for terms and conditions.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "gnss_provider.hpp"
#include "imu_provider.hpp"
#include "visual_provider.hpp"
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
    struct Config
    {
        std::string fixed_frame_id = "world";
        std::string body_frame_id = "";       ///< Empty means autodetect (IMU frame)

        Config()
        {
            ros::NodeHandle node("~");

            (void)node.getParam("fixed_frame_id", fixed_frame_id);
            (void)node.getParam("body_frame_id", body_frame_id);

            ROS_INFO("Filter:\n"
                     "\t~fixed_frame_id = %s\n"
                     "\t~body_frame_id = %s",
                     fixed_frame_id.c_str(),
                     body_frame_id.empty() ? "<derive from IMU>" : body_frame_id.c_str());
        }
    } const config_;

    mutable tf2_ros::TransformBroadcaster pub_tf_;
    mutable ros::Publisher pub_odometry_;
    mutable ros::Publisher pub_imu_;

    Filter filter_;
    GNSSProvider gnss_provider_;
    IMUProvider imu_provider_;
    VisualProvider visual_provider_;

    void publishEstimations(const std::string& body_frame_id) const
    {
        /*
         * Main filter output - odometry message
         */
        nav_msgs::Odometry odom;
        odom.header.frame_id = config_.fixed_frame_id;     // Fixed frame name can be configured
        odom.header.stamp.fromSec(filter_.getTimestamp());

        odom.child_frame_id = config_.body_frame_id.empty() ? body_frame_id : config_.body_frame_id;

        const auto pose = filter_.getOutputPose();
        odom.pose.covariance = matrixEigenToMsg(pose.position_orientation_cov);
        tf::quaternionEigenToMsg(pose.orientation, odom.pose.pose.orientation);
        tf::pointEigenToMsg(pose.position, odom.pose.pose.position);

        const auto twist = filter_.getOutputTwistInIMUFrame();
        odom.twist.covariance = matrixEigenToMsg(twist.linear_angular_cov);
        tf::vectorEigenToMsg(twist.angular, odom.twist.twist.angular);
        tf::vectorEigenToMsg(twist.linear,  odom.twist.twist.linear);

        pub_odometry_.publish(odom);

        /*
         * IMU message
         */
        sensor_msgs::Imu imu;
        imu.header.frame_id = odom.child_frame_id;
        imu.header.stamp.fromSec(filter_.getTimestamp());

        imu.angular_velocity_covariance = matrixEigenToMsg(Matrix3(twist.linear_angular_cov.block<3, 3>(3, 3)));
        imu.orientation_covariance = matrixEigenToMsg(Matrix3(pose.position_orientation_cov.block<3, 3>(3, 3)));

        imu.orientation = odom.pose.pose.orientation;
        tf::vectorEigenToMsg(twist.angular, imu.angular_velocity);

        {
            const auto accel_and_cov = filter_.getOutputAcceleration();
            tf::vectorEigenToMsg(accel_and_cov.first, imu.linear_acceleration);
            imu.linear_acceleration_covariance = matrixEigenToMsg(accel_and_cov.second);
        }

        pub_imu_.publish(imu);

        /*
         * World --> IMU transform
         */
        geometry_msgs::TransformStamped tf;
        tf.header = odom.header;
        tf.child_frame_id = odom.child_frame_id;

        tf.transform.rotation = odom.pose.pose.orientation;
        tf::vectorEigenToMsg(pose.position, tf.transform.translation);

        pub_tf_.sendTransform(tf);
    }

    void cbImu(const IMUSample& sample, const sensor_msgs::Imu& msg)
    {
        if (filter_.getTimestamp() >= (sample.timestamp.toSec() + 0.3))  // TODO: get rid of this later
        {
            ROS_WARN_THROTTLE(1, "IMU update from the past [%f sec]",
                              filter_.getTimestamp() - sample.timestamp.toSec());
            return;
        }

        /*
         * Filter update
         * TODO: proper initialization
         */
        if (!filter_.isInitialized())
        {
//            filter_.initialize(sample.timestamp.toSec(), quaternionFromEuler(Vector3::Zero()));
            filter_.initialize(sample.timestamp.toSec(),
                               quaternionFromEuler(Vector3(0, 0, M_PI / 2.0)) * sample.orientation);
            return;
        }

        filter_.performTimeUpdate(sample.timestamp.toSec());
        filter_.performAccelUpdate(sample.accel, sample.accel_covariance);
        filter_.performGyroUpdate(sample.gyro, sample.gyro_covariance);

        const std::string body_frame_id = msg.header.frame_id.empty() ? "base_link" : msg.header.frame_id;
        publishEstimations(body_frame_id);
    }

    void cbGnss(const GNSSLocalPosVel& local, const gps_common::GPSFix&)
    {
        if (!filter_.isInitialized())
        {
            ROS_WARN_THROTTLE(1, "GNSS update skipped - not inited yet");
            return;
        }

        filter_.performTimeUpdate(local.timestamp.toSec());
        filter_.performGNSSPosUpdate(local.position, local.position_covariance);
        filter_.performGNSSVelUpdate(local.velocity, local.velocity_covariance);
    }

    void cbVisual(const VisualSample& sample, const CameraTransform& transform)
    {
        (void)transform;
        if (!filter_.isInitialized())
        {
            ROS_WARN_THROTTLE(1, "Visual update skipped - not inited yet");
            return;
        }

        if (!sample.valid)
        {
            filter_.invalidateVisOffsets();
            ROS_WARN_THROTTLE(1, "Visual offsets invalidated");
            return;
        }

        filter_.performTimeUpdate(sample.timestamp.toSec());

        filter_.performVisPosUpdate(sample.position, Matrix3(sample.position_orientation_cov.block<3, 3>(0, 0)));
        filter_.performVisAttUpdate(sample.orientation, Matrix3(sample.position_orientation_cov.block<3, 3>(3, 3)));
    }

public:
    FilterWrapper(ros::NodeHandle& node)
        : gnss_provider_(node, "gnss")
        , imu_provider_(node, "imu")
        , visual_provider_(node, "visual_odom")
    {
        gnss_provider_.on_sample = std::bind(&FilterWrapper::cbGnss, this,
                                             std::placeholders::_1, std::placeholders::_2);

        imu_provider_.on_sample = std::bind(&FilterWrapper::cbImu, this,
                                            std::placeholders::_1, std::placeholders::_2);

        visual_provider_.on_sample = std::bind(&FilterWrapper::cbVisual, this,
                                               std::placeholders::_1, std::placeholders::_2);

        pub_odometry_ = node.advertise<nav_msgs::Odometry>("out_odom", 10);
        pub_imu_ = node.advertise<sensor_msgs::Imu>("out_imu", 10);

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
