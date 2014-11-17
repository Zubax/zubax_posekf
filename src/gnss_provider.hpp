/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Please refer to the file LICENSE for terms and conditions.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#pragma once

#include <functional>
#include <ros/ros.h>
#include <gps_common/GPSFix.h>
#include <eigen_conversions/eigen_msg.h>
#include "linear_algebra.hpp"
#include "debug_publisher.hpp"
#include "exception.hpp"
#include "mathematica.hpp"

namespace zubax_posekf
{
/**
 * This is the representation that can be directly used by the filter.
 */
struct GNSSLocalPosVel
{
    ros::Time timestamp;

    Vector3 position;
    Matrix3 position_covariance;

    Vector3 velocity;
    Matrix3 velocity_covariance;

    GNSSLocalPosVel()
    {
        position.setZero();
        position_covariance.setZero();

        velocity.setZero();
        velocity_covariance.setZero();
    }

    bool isValid() const { return timestamp.isValid(); }
};

/**
 * Geo coordinates expressed as longitude and latitude degrees (not radians).
 */
struct GeoLonLat
{
    Scalar lon = 0.0;
    Scalar lat = 0.0;

    GeoLonLat() { }

    GeoLonLat(Scalar arg_lon, Scalar arg_lat)
        : lon(arg_lon)
        , lat(arg_lat)
    { }
};

/**
 * This class converts between spherical geo coordinates and metric offset around certain origin.
 */
class GeoPositionConverter
{
    const Scalar MetersPerLatDegree = (110567.0 + 111699.0) * 0.5; // average from equator to poles
    const Scalar MetersPerLonDegreeOnEquator = 111321.0;

    const GeoLonLat origin_;

    Scalar latM2Deg(Scalar m) const { return m / MetersPerLatDegree; }
    Scalar latDeg2M(Scalar d) const { return d * MetersPerLatDegree; }

    Scalar lonMPerDeg(Scalar lat) const
    {
        return MetersPerLonDegreeOnEquator * ((90. - std::min(std::abs(lat), Scalar(89.999999))) / 90.);
    }

    Scalar lonM2Deg(Scalar lat, Scalar m) const { return m / lonMPerDeg(lat); }
    Scalar lonDeg2M(Scalar lat, Scalar d) const { return d * lonMPerDeg(lat); }

public:
    GeoPositionConverter(const GeoLonLat& arg_origin)
        : origin_(arg_origin)
    { }

    Vector<2> convertGeoToMetric(const GeoLonLat& geo) const
    {
        Vector<2> metric;
        metric[1] = latDeg2M(geo.lat - origin_.lat);          // lat, Y
        metric[0] = lonDeg2M(geo.lat, geo.lon - origin_.lon); // lon, X
        return metric;
    }

    GeoLonLat convertMetricToGeo(const Vector<2>& metric) const
    {
        GeoLonLat geo;
        geo.lat = latM2Deg(metric[1]) + origin_.lat;          // lat, Y
        geo.lon = lonM2Deg(geo.lat, metric[0]) + origin_.lon; // lon, X
        return geo;
    }
};

/**
 * This class obtains GNSS measurements from the specified ROS topic and converts them to proper form.
 */
class GNSSProvider
{
    const Scalar DefaultTrackStdevDeg = 30.0;
    const Scalar DefaultClimbRateStdevMS = 4.0;
    const Scalar MaxSaneSpeedMS = 515.0;

    ros::Subscriber sub_gnss_;
    DebugPublisher pub_debug_;

    GeoLonLat origin_;
    bool origin_set_ = false;

    GNSSLocalPosVel last_sample_;

    struct Config
    {
        int min_sats_to_init_origin = 0;
        double max_err_horz_to_init_origin = 40.0;

        Config()
        {
            ros::NodeHandle node("~gnss");

            (void)node.getParam("min_sats_to_init_origin", min_sats_to_init_origin);
            (void)node.getParam("max_err_horz_to_init_origin", max_err_horz_to_init_origin);

            ROS_INFO("GNSS Provider:\n"
                     "\t~gnss/min_sats_to_init_origin     = %d\n"
                     "\t~gnss/max_err_horz_to_init_origin = %f",
                     min_sats_to_init_origin,
                     max_err_horz_to_init_origin);
        }
    } const config_;

    std::pair<Vector3, Matrix3> computeLocalPositionWithCovariance(const gps_common::GPSFix& msg) const
    {
        enforce("GNSS origin is not set", origin_set_);

        const auto metric = GeoPositionConverter(origin_).convertGeoToMetric(GeoLonLat(msg.longitude, msg.latitude));

        Vector3 pos;
        pos[0] = metric[0];
        pos[1] = metric[1];
        pos[2] = msg.altitude;

        Matrix3 cov;
        cov.setZero();
        if (msg.position_covariance_type == msg.COVARIANCE_TYPE_UNKNOWN)
        {
            cov(0, 0) = msg.err_horz * msg.err_horz;
            cov(1, 1) = msg.err_horz * msg.err_horz;
            cov(2, 2) = msg.err_vert * msg.err_vert;
        }
        else
        {
            cov = matrixMsgToEigen<3, 3>(msg.position_covariance);
            cov = (cov + cov.transpose()) * 0.5;          // Ensure it's symmetric
        }

        enforce("GNSS position", cov.sum() > 0);

        return {pos, cov};
    }

    std::pair<Vector3, Matrix3> computeVelocityVectorWithCovariance(const gps_common::GPSFix& msg) const
    {
        using namespace mathematica;

        const Scalar gnssTrack = msg.track * Degree;
        const Scalar gnssSpeed = msg.speed;
        const Scalar gnssClimb = msg.climb;

        enforce("GNSS track, speed, climb",
                (Abs(gnssTrack) <= 2.0 * Pi) &&
                (Abs(gnssSpeed) < MaxSaneSpeedMS) &&
                (Abs(gnssClimb) < MaxSaneSpeedMS));

        const auto vel = List(List(gnssSpeed * Sin(gnssTrack)),  // Lon
                              List(gnssSpeed * Cos(gnssTrack)),  // Lat
                              List(gnssClimb));                  // Climb

        const Scalar err_track_rad = ((msg.err_track > 0) ? msg.err_track : DefaultTrackStdevDeg) * Degree;

        const auto Rpolar = List(List(Power(err_track_rad, 2), 0, 0),  // stdev --> covariance matrix
                                 List(0, Power(msg.err_speed, 2), 0),
                                 List(0, 0, Power((msg.err_climb > 0) ? msg.err_climb : DefaultClimbRateStdevMS, 2)));

        if ((Rpolar(0, 0) <= 0) || (Rpolar(1, 1) <= 0) || (Rpolar(2, 2) <= 0))
        {
            std::ostringstream os;
            os << "GNSS R polar:\n" << Rpolar;
            throw Exception(os.str());
        }

        // Convariance transformation Jacobian
        const auto Gpolar = List(List(gnssSpeed * Cos(gnssTrack), Sin(gnssTrack), 0),
                                 List(-(gnssSpeed * Sin(gnssTrack)), Cos(gnssTrack), 0),
                                 List(0, 0, 1));

        const Matrix3 R = Gpolar * Rpolar * Gpolar.transpose();

        if ((R(0, 0) <= 0) || (R(1, 1) <= 0) || (R(2, 2) <= 0) || !std::isfinite(R.sum()))
        {
            std::ostringstream os;
            os << "GNSS R vel:\n" << R << "\nGNSS R polar:\n" << Rpolar;
            throw Exception(os.str());
        }

        return {vel, R};
    }

    void cbGnss(const gps_common::GPSFix& msg)
    {
        /*
         * Initialize origin if necessary
         */
        if (!origin_set_)
        {
            if (msg.status.satellites_used >= config_.min_sats_to_init_origin &&
                msg.err_horz < config_.max_err_horz_to_init_origin &&
                msg.status.status >= msg.status.STATUS_FIX)
            {
                origin_.lon = msg.longitude;
                origin_.lat = msg.latitude;
                origin_set_ = true;
                ROS_INFO("GNSS Provider: Origin at lon=%f, lat=%f", origin_.lon, origin_.lat);
            }
            else
            {
                ROS_INFO_THROTTLE(1, "GNSS Provider: Can't initialize origin: nsats=%d, err_horz=%f, status=%d",
                                  msg.status.satellites_used, msg.err_horz, static_cast<int>(msg.status.status));
                return;
            }
        }

        /*
         * Position and covariance
         */
        const auto pos_and_cov = computeLocalPositionWithCovariance(msg);
        pub_debug_.publish("gnss_pos", pos_and_cov.first);
        pub_debug_.publish("gnss_pos_cov", pos_and_cov.second);

        /*
         * Velocity and covariance
         */
        const auto vel_and_cov = computeVelocityVectorWithCovariance(msg);
        pub_debug_.publish("gnss_vel", vel_and_cov.first);
        pub_debug_.publish("gnss_vel_cov", vel_and_cov.second);

        /*
         * Output
         */
        last_sample_.timestamp = msg.header.stamp;

        last_sample_.velocity = vel_and_cov.first;
        last_sample_.velocity_covariance = vel_and_cov.second;

        last_sample_.position = pos_and_cov.first;
        last_sample_.position_covariance = pos_and_cov.second;

        ROS_ASSERT(origin_set_);
        if (on_sample)
        {
            on_sample(last_sample_, msg);
        }
    }

public:
    GNSSProvider(ros::NodeHandle& node, const std::string topic)
    {
        sub_gnss_ = node.subscribe(topic, 10, &GNSSProvider::cbGnss, this);
    }

    GNSSLocalPosVel getLastSample() const { return last_sample_; }

    GeoLonLat getOrigin() const { return origin_; }
    bool isOriginSet() const { return origin_set_; }

    std::function<void (const GNSSLocalPosVel&, const gps_common::GPSFix&)> on_sample;
};

}
