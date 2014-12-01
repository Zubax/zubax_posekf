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
#include "measurement.hpp"

namespace zubax_posekf
{
/**
 * This is the representation that can be directly used by the filter.
 */
struct GNSSLocalPosVel : public Measurement
{
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
    static constexpr Scalar EarthRadiusMeters = 6372797.0;

    const GeoLonLat origin_;

    Scalar greatCircleToMetric(Scalar angle) const
    {
        // m = (earth_perimeter / 2PI) * angle
        // m = ((2PI * earth_radius) / 2PI) * angle
        // m = earth_radius * angle
        return EarthRadiusMeters * angle;
    }

    Scalar greatCircleAngle(const Vector3& a, const Vector3& b) const
    {
        using namespace std;
        Scalar lat1, lon1, lat2, lon2;
        decomposeLatLon(a, b, lat1, lon1, lat2, lon2);
        Scalar t1 = sin((lat2-lat1) / 2.0);
        t1 *= t1;
        Scalar t2 = sin((lon2-lon1) / 2.0);
        t2 *= t2;
        return 2.0 * asin(sqrt(t1 + cos(lat1) * cos(lat2) * t2));
    }

    void decomposeLatLon(const Vector3& a, Scalar& lat1, Scalar& lon1) const
    {
        static const Scalar deg2rad = M_PI / 180.0;
        lat1 = a.y() * deg2rad;
        lon1 = a.x() * deg2rad;
    }

    void decomposeLatLon(const Vector3& a, const Vector3& b,
                         Scalar& lat1, Scalar& lon1, Scalar& lat2, Scalar& lon2) const
    {
        decomposeLatLon(a, lat1, lon1);
        decomposeLatLon(b, lat2, lon2);
    }

    Scalar genericDistance(const Vector3& a, const Vector3& b, bool plane) const
    {
        const Scalar Precision = 1e-9;

        Scalar d_xy = 0.0;

        if ((std::abs(a.x() - b.x()) > Precision) ||
            (std::abs(a.y() - b.y()) > Precision))
        {
            const Scalar gca = greatCircleAngle(a, b);
            d_xy = greatCircleToMetric(gca);
        }

        if (plane)
        {
            return d_xy;
        }
        else
        {
            const Scalar d_z = a.z() - b.z();
            return std::sqrt(d_xy * d_xy + d_z * d_z);
        }
    }

public:
    GeoPositionConverter(const GeoLonLat& arg_origin)
        : origin_(arg_origin)
    { }

    Vector<2> convertGeoToMetric(const GeoLonLat& geo) const
    {
        const Vector3 origin_vector(origin_.lon, origin_.lat, 0.0);

        Scalar x = genericDistance(origin_vector, Vector3(geo.lon, origin_.lat, 0.0), true);
        if (geo.lon < origin_.lon)
        {
            x *= -1.0;
        }

        Scalar y = genericDistance(origin_vector, Vector3(origin_.lon, geo.lat, 0.0), true);
        if (geo.lat < origin_.lat)
        {
            y *= -1.0;
        }

        return Vector<2>(x, y);
    }
};

/**
 * This class obtains GNSS measurements from the specified ROS topic and converts them to proper form.
 */
class GNSSProvider
{
    const Scalar DefaultTrackStdevDeg = 30.0;
    const Scalar DefaultClimbRateStdevMS = 4.0;
    const Scalar MaxSaneSpeedMS = 515.0;        // ITAR limit, used for sanity checks
    const Scalar MinSpeedMS = 1e-3;

    ros::Subscriber sub_gnss_;
    DebugPublisher pub_debug_;

    GeoLonLat origin_;
    Scalar origin_altitude_ = 0.0;
    bool origin_set_ = false;

    GNSSLocalPosVel last_sample_;

    struct Config
    {
        int min_sats = 0;
        int min_sats_to_init_origin = 0;
        double max_err_horz_to_init_origin = 40.0;
        double position_cov_mult = 1.0;            ///< TODO: heuristic covariance estimation
        double velocity_cov_mult = 1.0;

        Config()
        {
            ros::NodeHandle node("~gnss");

            (void)node.getParam("min_sats", min_sats);
            (void)node.getParam("min_sats_to_init_origin", min_sats_to_init_origin);
            (void)node.getParam("max_err_horz_to_init_origin", max_err_horz_to_init_origin);
            (void)node.getParam("position_cov_mult", position_cov_mult);
            (void)node.getParam("velocity_cov_mult", velocity_cov_mult);

            ROS_INFO("GNSS Provider:\n"
                     "\t~gnss/min_sats                    = %d\n"
                     "\t~gnss/min_sats_to_init_origin     = %d\n"
                    "\t~gnss/max_err_horz_to_init_origin = %f\n"
                    "\t~gnss/position_cov_mult = %f\n"
                    "\t~gnss/velocity_cov_mult = %f",
                     min_sats,
                     min_sats_to_init_origin,
                     max_err_horz_to_init_origin,
                     position_cov_mult,
                     velocity_cov_mult);
        }
    } const config_;

    std::pair<Vector3, Matrix3> computeLocalPositionWithCovariance(const gps_common::GPSFix& msg) const
    {
        enforce("GNSS origin is not set", origin_set_);

        const auto metric = GeoPositionConverter(origin_).convertGeoToMetric(GeoLonLat(msg.longitude, msg.latitude));

        Vector3 pos;
        pos[0] = metric[0];
        pos[1] = metric[1];
        pos[2] = msg.altitude - origin_altitude_;

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

        return {pos, cov * config_.position_cov_mult};
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
        Matrix3 Gpolar;
        {
            /*
             * Special case: if speed ~= track ~= 0, the Jacobian produces zero variance for the longitudinal speed:
             *  0  0   0
             *  0 svar 0
             *  0  0  cvar
             * Zero variances aren't allowed by the filter.
             */
            const Scalar speed = (gnssSpeed > MinSpeedMS) ? gnssSpeed : MinSpeedMS; // Work around
            Gpolar = List(List(  speed * Cos(gnssTrack),  Sin(gnssTrack), 0),
                          List(-(speed * Sin(gnssTrack)), Cos(gnssTrack), 0),
                          List(0,                         0,              1));
        }

        const Matrix3 R = Gpolar * Rpolar * Gpolar.transpose();

        if ((R(0, 0) <= 0) || (R(1, 1) <= 0) || (R(2, 2) <= 0) || !std::isfinite(R.sum()))
        {
            std::ostringstream os;
            os << "GNSS R vel:\n" << R << "\nGNSS R polar:\n" << Rpolar << "\nGNSS G polar:\n" << Gpolar
                << "\ntrack=" << gnssTrack << "\nspeed=" << gnssSpeed;
            throw Exception(os.str());
        }

        return {vel, R * config_.velocity_cov_mult};
    }

    void cbGnss(const gps_common::GPSFix& msg)
    {
        /*
         * Drop invalid messages
         */
        {
            const bool has_fix = msg.status.status >= msg.status.STATUS_FIX;
            const bool sats_ok = msg.status.satellites_used >= config_.min_sats;
            if (!has_fix || !sats_ok)
            {
                ROS_DEBUG("GNSS Provider: Message dropped: status=%d, nsats=%d",
                          msg.status.status, msg.status.satellites_used);
                return;
            }
        }

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
                origin_altitude_ = msg.altitude;
                origin_set_ = true;
                ROS_INFO("GNSS Provider: Origin at lon=%f, lat=%f, alt=%f",
                         origin_.lon, origin_.lat, origin_altitude_);
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

    std::function<void (const GNSSLocalPosVel&, const gps_common::GPSFix&)> on_sample;
};

}
