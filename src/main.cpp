/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Please refer to the file LICENSE for terms and conditions.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include "gnss_provider.hpp"
#include "imu_provider.hpp"
#include "visual_provider.hpp"
#include "filter.hpp"
#include "linear_algebra.hpp"
#include "exception.hpp"
#include "stopwatch.hpp"

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
        std::string gnss_odom_frame_id = "";  ///< Empty means disabled

        Config()
        {
            ros::NodeHandle node("~");

            (void)node.getParam("fixed_frame_id", fixed_frame_id);
            (void)node.getParam("body_frame_id", body_frame_id);
            (void)node.getParam("gnss_odom_frame_id", gnss_odom_frame_id);

            ROS_INFO("Filter:\n"
                     "\t~fixed_frame_id     = %s\n"
                     "\t~body_frame_id      = %s\n"
                     "\t~gnss_odom_frame_id = %s",
                     fixed_frame_id.c_str(),
                     body_frame_id.empty() ? "<derive from IMU>" : body_frame_id.c_str(),
                     gnss_odom_frame_id.empty() ? "<disabled>" : gnss_odom_frame_id.c_str());
        }
    } const config_;

    mutable tf2_ros::TransformBroadcaster pub_tf_;
    mutable ros::Publisher pub_odometry_;
    mutable ros::Publisher pub_gnss_odom_;
    mutable ros::Publisher pub_marker_;

    mutable DebugPublisher debug_pub_;

    Filter filter_;
    GNSSProvider gnss_provider_;
    IMUProvider imu_provider_;
    VisualProvider visual_provider_;

    void publishDebugInfo() const
    {
        const auto ts_state = filter_.getCurrentState();
        const auto state = ts_state.second;

        debug_pub_.publish("x", state.getStateVector().x);
        debug_pub_.publish("P_diag", state.getCovariance().diagonal());

        debug_pub_.publish("pvw", state.getStateVector().pvw());
        debug_pub_.publish("qvw_rpy_deg",
                           Vector3(quaternionToEuler(state.getStateVector().qvw()) / mathematica::Degree));

        debug_pub_.publish("qwi_rpy_deg",
                           Vector3(quaternionToEuler(state.getStateVector().qwi()) / mathematica::Degree));

        debug_pub_.publish("dt", (ts_state.first - filter_.getPreviousState().first).toSec());
    }

    void publishEstimations(const std::string& body_frame_id) const
    {
        /*
         * Main filter output - odometry message
         */
        const auto ts_state = filter_.getCurrentState();

        nav_msgs::Odometry odom;
        odom.header.frame_id = config_.fixed_frame_id;     // Fixed frame name can be configured
        odom.header.stamp = ts_state.first;

        odom.child_frame_id = config_.body_frame_id.empty() ? body_frame_id : config_.body_frame_id;

        const auto pose = ts_state.second.getPose();
        odom.pose.covariance = matrixEigenToMsg(pose.position_orientation_cov);
        tf::quaternionEigenToMsg(pose.orientation, odom.pose.pose.orientation);
        tf::pointEigenToMsg(pose.position, odom.pose.pose.position);

        const auto twist = ts_state.second.getTwistInIMUFrame();
        odom.twist.covariance = matrixEigenToMsg(twist.linear_angular_cov);
        tf::vectorEigenToMsg(twist.angular, odom.twist.twist.angular);
        tf::vectorEigenToMsg(twist.linear,  odom.twist.twist.linear);

        pub_odometry_.publish(odom);

        /*
         * World --> IMU transform
         */
        {
            geometry_msgs::TransformStamped tf;
            tf.header = odom.header;
            tf.child_frame_id = odom.child_frame_id;

            tf.transform.rotation = odom.pose.pose.orientation;
            tf::vectorEigenToMsg(pose.position, tf.transform.translation);

            pub_tf_.sendTransform(tf);
        }

        /*
         * Visualization
         */
        if (pub_marker_.getNumSubscribers() > 0)
        {
            pub_marker_.publish(makeVectorVisualization(odom.header.stamp, body_frame_id, 1,
                                                        twist.linear, {1, 0, 1, 0.6}));
            pub_marker_.publish(makeVectorVisualization(odom.header.stamp, body_frame_id, 2,
                                                        ts_state.second.getAcceleration().first, {1, 1, 0, 0.3}));
            pub_marker_.publish(makeVectorVisualization(odom.header.stamp, config_.fixed_frame_id, 3,
                                                        ts_state.second.getTwistInWorldFrame().linear, {1, 1, 1}));
        }
    }

    void cbImu(const IMUSample& sample, const sensor_msgs::Imu& msg)
    {
        /*
         * Filter update
         * TODO: proper initialization
         */
        if (!filter_.isInitialized())
        {
            if (!gnss_provider_.getLastSample().isValid())
            {
                return;
            }
            filter_.initialize(sample.timestamp,
                               quaternionFromEuler(Vector3(0, 0, M_PI / 2.0)) * sample.orientation,
                               gnss_provider_.getLastSample().position,
                               gnss_provider_.getLastSample().velocity);
            return;
        }

        filter_.update(std::make_shared<IMUSample>(sample));

        /*
         * Filter output at IMU rate
         */
        const std::string body_frame_id = msg.header.frame_id.empty() ? "base_link" : msg.header.frame_id;
        publishEstimations(body_frame_id);

        /*
         * Debug info
         */
        publishDebugInfo();
        debug_pub_.publish("imu_rpy_deg", quaternionToEuler(sample.orientation) / mathematica::Degree);
        debug_pub_.publish("rewind_imu", filter_.computeHowManyUpdatesHappenedSince(sample.timestamp));
    }

    void cbGnss(const GNSSLocalPosVel& local, const gps_common::GPSFix&)
    {
        if (!filter_.isInitialized())
        {
            ROS_WARN_THROTTLE(1, "GNSS update skipped - not inited yet");
            return;
        }

        filter_.update(std::make_shared<GNSSLocalPosVel>(local));

        /*
         * Display this GNSS measurement as an Odometry message
         */
        if (!config_.gnss_odom_frame_id.empty())
        {
            nav_msgs::Odometry odom;
            odom.header.frame_id = config_.fixed_frame_id;
            odom.header.stamp = local.timestamp;
            odom.child_frame_id = config_.gnss_odom_frame_id;
            tf::pointEigenToMsg(local.position, odom.pose.pose.position);
            tf::quaternionEigenToMsg(Quaternion::FromTwoVectors(Vector3(1, 0, 0), local.velocity),
                                     odom.pose.pose.orientation);
            pub_gnss_odom_.publish(odom);
        }

        debug_pub_.publish("rewind_gnss", filter_.computeHowManyUpdatesHappenedSince(local.timestamp));
    }

    void cbVisual(const VisualSample& sample, const CameraTransform& transform)
    {
        (void)transform;
        if (!filter_.isInitialized())
        {
            ROS_WARN_THROTTLE(1, "Visual update skipped - not inited yet");
            return;
        }

        if (!sample.pose_valid)
        {
            filter_.invalidateVisualOffsetsSince(sample.timestamp);
        }

        filter_.update(std::make_shared<VisualSample>(sample));

        debug_pub_.publish("rewind_vis", filter_.computeHowManyUpdatesHappenedSince(sample.timestamp));
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
        pub_gnss_odom_ = node.advertise<nav_msgs::Odometry>("gnss_odom", 10);
        pub_marker_ = node.advertise<visualization_msgs::Marker>("visualization_marker", 10);

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
