/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Please refer to the file LICENSE for terms and conditions.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <eigen3/Eigen/Eigen>
#include <cmath>
#include <memory>
#include "mathematica.hpp"
#include "debug_publisher.hpp"
#include "linear_algebra.hpp"
#include "exception.hpp"
#include "history_keeper.hpp"
#include "state_vector_autogenerated.hpp"
#include "imu_provider.hpp"
#include "gnss_provider.hpp"
#include "visual_provider.hpp"

namespace zubax_posekf
{
/**
 * Filter output - twist; compatible with geometry_msgs/TwistWithCovariance.
 * - Velocity of IMU in the world frame, transformed into the IMU frame
 * - Angular velocity of IMU in the IMU frame
 * - Covariance: x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis
 */
struct Twist
{
    Vector3 linear;
    Vector3 angular;
    Matrix<6, 6> linear_angular_cov;

    Twist()
    {
        linear.setZero();
        angular.setZero();
        linear_angular_cov.setZero();
    }
};

/**
 * Filter output - pose; compatible with geometry_msgs/PoseWithCovariance.
 * - Position of IMU in the world frame
 * - Orientation of IMU in the world frame
 * - Covariance: x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis
 */
struct Pose
{
    Vector3 position;
    Quaternion orientation;
    Matrix<6, 6> position_orientation_cov;

    Pose()
    {
        position.setZero();
        orientation.setIdentity();
        position_orientation_cov.setZero();
    }
};

/**
 * Filter x and P with management functions.
 */
class FilterStateWithCovariance
{
    StateVector x;
    Matrix<StateVector::Size, StateVector::Size> P;

    static Vector3 constrainPerComponent(Vector3 vec, const Scalar limit, const char* name)
    {
        for (int i = 0; i < 3; i++)
        {
            if (!checkRangeAndConstrainSymmetric(vec[i], limit))
            {
                ROS_WARN_THROTTLE(1, "Too high - %s", name);
            }
        }
        return vec;
    }

    /**
     * Normalizes the current state and runs a simple sanity check.
     * @throws zubax_posekf::Exception.
     */
    void normalizeAndCheck()
    {
        const Scalar MaxGyroDrift = 0.1;
        const Scalar MaxAccelDrift = 2.0;
        const Scalar MaxAngularVelocity = M_PI * 2;
        const Scalar MaxCovariance = 1e9;
        const Scalar MinVariance = 1e-9;

        /*
         * x
         */
        x.normalize();

        x.bw(constrainPerComponent(x.bw(), MaxGyroDrift, "gyro drift"));
        x.ba(constrainPerComponent(x.ba(), MaxAccelDrift, "accel drift"));
        x.w(constrainPerComponent(x.w(), MaxAngularVelocity, "angular velocity"));

        /*
         * P
         */
        if (!validateAndFixCovarianceMatrix(P, MaxCovariance, MinVariance))
        {
            ROS_ERROR_STREAM_THROTTLE(1, "Matrix P has been fixed\n" << P.format(Eigen::IOFormat(2)));
        }

        /*
         * Data validness
         */
        enforce("Non-finite states",
                std::isfinite(x.x.sum()) &&
                std::isfinite(P.sum()));
    }

public:
    FilterStateWithCovariance()
    {
        // Initial state - zero everything, null rotations
        x.qwi(Quaternion(1, 0, 0, 0));
        x.qvw(Quaternion(1, 0, 0, 0));
        x.lambda = 1.0;

        // Initial P
        P.setZero();
        P = StateVector::Pinitdiag().asDiagonal();
    }

    FilterStateWithCovariance(const Vector<StateVector::Size>& arg_x,
                              const Matrix<StateVector::Size, StateVector::Size>& arg_P)
        : x(arg_x)
        , P(arg_P)
    { }

    void initializeOrientationPositionVelocity(const Quaternion& orientation,
                                               const Vector3& position,
                                               const Vector3& velocity)
    {
        x.qwi(orientation);
        x.pwi(position);
        x.vwi(velocity);

        normalizeAndCheck();
    }

    /**
     * Modifies the current state with a new measurement.
     * @param y                 Measurement residual
     * @param R                 Measurement error covariance
     * @param H                 Observation matrix
     * @param source_name       Name of the update equation, used for logging
     */
    template <int NumDims>
    void performMeasurementUpdate(const Vector<NumDims>& y,
                                  const Matrix<NumDims, NumDims>& R,
                                  const Matrix<NumDims, StateVector::Size>& H,
                                  const char* const source_name)
    {
        /*
         * Ensure that R is symmetric, then validate
         */
        const Matrix<NumDims, NumDims> R_sym = 0.5 * (R + R.transpose());
        {
            bool R_sym_ok = true;
            for (int i = 0; i < NumDims; i++)
            {
                if (R_sym(i, i) <= 0)
                {
                    R_sym_ok = false;
                    break;
                }
            }

            if (!R_sym_ok || !std::isfinite(R_sym.sum()))
            {
                std::ostringstream os;
                os << "Measurement R_sym [" << source_name << "]:\n" << R_sym;
                throw Exception(os.str());
            }
        }

        /*
         * Compute S inverse with validation
         */
        Matrix<NumDims, NumDims> S_inv;
        {
            const Matrix<NumDims, NumDims> S = H * P * H.transpose() + R_sym;
            bool s_is_invertible = false;
            S.computeInverseWithCheck(S_inv, s_is_invertible);
            if (!s_is_invertible)
            {
                std::ostringstream os;
                os << "S is not invertible (numerical failure?) [" << source_name << "]:\n" << S
                    << "\nR_sym:\n" << R_sym;
                throw Exception(os.str());
            }
        }

        /*
         * Kalman update equations
         * Joseph form is used instead of the simplified form to improve numerical stability
         */
        const auto K = static_cast<Matrix<StateVector::Size, NumDims> >(P * H.transpose() * S_inv);

        x.x = x.x + K * y;

        const Matrix<StateVector::Size, StateVector::Size> IKH = decltype(P)::Identity() - K * H;
        P = IKH * P * IKH.transpose() + K * R_sym * K.transpose();

        normalizeAndCheck();
    }

    /**
     * Modifies the current state according to the prediction model.
     * @param dt        Time delta from the current state
     * @param Q         Process noise matrix
     */
    void propagate(Scalar dt, const Matrix<StateVector::Size, StateVector::Size>& Q)
    {
        ROS_ASSERT(dt > 0);

        const auto F = x.F(dt);

        x.x = x.f(dt);

        P = F * P * F.transpose() + Q;

        normalizeAndCheck();
    }

    /**
     * Pose, compatible with geometry_msgs/PoseWithCovariance
     */
    Pose getPose() const
    {
        Pose out;

        out.position = x.pwi();
        out.orientation = x.qwi();

        const Matrix<3, 4> G = quaternionToEulerJacobian(out.orientation);
        const Matrix4 C = P.block<4, 4>(StateVector::Idx::qwiw, StateVector::Idx::qwiw);

        out.position_orientation_cov.block<3, 3>(0, 0) = P.block<3, 3>(StateVector::Idx::pwix, StateVector::Idx::pwix);
        out.position_orientation_cov.block<3, 3>(3, 3) = G * C * G.transpose();

        return out;
    }

    /**
     * Twist, compatible with geometry_msgs/TwistWithCovariance
     */
    Twist getTwistInIMUFrame() const
    {
        Twist out;
        out.linear = rotateVectorByQuaternion(x.vwi(), x.qwi().inverse());  // World --> IMU
        out.angular = x.w();

        const Matrix3 G = rotateVectorByQuaternionJacobian(x.qwi());
        const Matrix3 C = P.block<3, 3>(StateVector::Idx::vwix, StateVector::Idx::vwix);

        out.linear_angular_cov.block<3, 3>(0, 0) = G * C * G.transpose();
        out.linear_angular_cov.block<3, 3>(3, 3) = P.block<3, 3>(StateVector::Idx::wx,   StateVector::Idx::wx);

        return out;
    }

    /**
     * Twist in the world frame
     */
    Twist getTwistInWorldFrame() const
    {
        Twist out;
        out.linear = x.vwi();
        out.angular = x.w();

        out.linear_angular_cov.block<3, 3>(0, 0) = P.block<3, 3>(StateVector::Idx::vwix, StateVector::Idx::vwix);
        out.linear_angular_cov.block<3, 3>(3, 3) = P.block<3, 3>(StateVector::Idx::wx,   StateVector::Idx::wx);

        return out;
    }

    /**
     * Gravity compensated acceleration in IMU frame with covariance
     */
    std::pair<Vector3, Matrix3> getAcceleration() const
    {
        return { x.a(), P.block<3, 3>(StateVector::Idx::ax, StateVector::Idx::ax) };
    }

    const StateVector& getStateVector() const { return x; }

    const Matrix<StateVector::Size, StateVector::Size>& getCovariance() const { return P; }

    Matrix<StateVector::Size, StateVector::Size>& getCovariance() { return P; }
};

/**
 * The business end.
 */
class Filter
{
    const Scalar HistoryLengthSec = 5.0;        ///< Maximum measurement delay

    HistoryKeeper<FilterStateWithCovariance> states_;
    HistoryKeeper<std::shared_ptr<Measurement> > measurements_;

    Matrix<StateVector::Size, StateVector::Size> computeQ(double dt) const
    {
        Vector<StateVector::Size> Qdiag = StateVector::Qmindiag();

        Qdiag *= dt * dt;

        return Qdiag.asDiagonal();
    }

    static void updateFromIMU(FilterStateWithCovariance& state, const IMUSample* meas)
    {
        {
            const Vector3 y = meas->accel - state.getStateVector().hacc();
            state.performMeasurementUpdate(y, meas->accel_covariance, state.getStateVector().Hacc(), "acc");
        }
        {
            const Vector3 y = meas->gyro - state.getStateVector().hgyro();
            state.performMeasurementUpdate(y, meas->gyro_covariance, state.getStateVector().Hgyro(), "gyro");
        }
    }

    static void updateFromGNSS(FilterStateWithCovariance& state, const GNSSLocalPosVel* meas)
    {
        {
            const Vector3 y = meas->position - state.getStateVector().hgnsspos();
            state.performMeasurementUpdate(y, meas->position_covariance, state.getStateVector().Hgnsspos(), "gnsspos");
        }
        {
            const Vector3 y = meas->velocity - state.getStateVector().hgnssvel();
            state.performMeasurementUpdate(y, meas->velocity_covariance, state.getStateVector().Hgnssvel(), "gnssvel");
        }
    }

    static void updateFromVisual(FilterStateWithCovariance& state, const VisualSample* meas)
    {
        if (meas->pose_valid)
        {
            const Vector3 y = meas->position - state.getStateVector().hvispos();
            const Matrix3 R = meas->position_orientation_cov.block<3, 3>(0, 0);
            state.performMeasurementUpdate(y, R, state.getStateVector().Hvispos(), "vispos");
        }

        if (meas->pose_valid)
        {
//            const Quaternion yq = meas->orientation * state.getStateVector().hvisatt().inverse();
//            const Vector<4> yvec(yq.w(), yq.x(), yq.y(), yq.z());

            const Vector<4> zvec(meas->orientation.w(),
                                 meas->orientation.x(),
                                 meas->orientation.y(),
                                 meas->orientation.z());

            const Quaternion xq = state.getStateVector().hvisatt();
            const Vector<4> xvec(xq.w(),
                                 xq.x(),
                                 xq.y(),
                                 xq.z());

            const Vector<4> yvec = zvec - xvec;

            const Matrix<4, 3> G = quaternionFromEulerJacobian(quaternionToEuler(meas->orientation));
            const auto R = static_cast<Matrix4>(G * meas->position_orientation_cov.block<3, 3>(3, 3) * G.transpose());

            state.performMeasurementUpdate(yvec, R, state.getStateVector().Hvisatt(), "visatt");
        }
    }

    static void updateImpl(FilterStateWithCovariance& state, const std::shared_ptr<Measurement>& virtual_meas)
    {
        if (auto meas = dynamic_cast<IMUSample*>(virtual_meas.get()))
        {
            updateFromIMU(state, meas);
        }
        else if (auto meas = dynamic_cast<GNSSLocalPosVel*>(virtual_meas.get()))
        {
            updateFromGNSS(state, meas);
        }
        else if (auto meas = dynamic_cast<VisualSample*>(virtual_meas.get()))
        {
            updateFromVisual(state, meas);
        }
        else
        {
            throw Exception("Unknown measurement type: " + std::string(typeid(*virtual_meas).name()));
        }
    }

public:
    bool isInitialized() const { return !states_.empty(); }

    /**
     * Initializes the filter with first apriori estimate.
     */
    void initialize(const ros::Time& timestamp,
                    const Quaternion& orientation,
                    const Vector3& position,
                    const Vector3& velocity)
    {
        ROS_ASSERT(states_.empty());

        FilterStateWithCovariance state;
        state.initializeOrientationPositionVelocity(orientation, position, velocity);
        states_.add(timestamp.toSec(), state);

        ROS_INFO_STREAM("Initial P:\n" << state.getCovariance().format(Eigen::IOFormat(2)));
        ROS_INFO_STREAM("Initial x:\n" << state.getStateVector().x.transpose().format(Eigen::IOFormat(2)));
    }

    /**
     * Returns the latest state of the filter.
     * @throws std::range_error if the filter is not yet initialized.
     */
    std::pair<ros::Time, const FilterStateWithCovariance&> getCurrentState() const
    {
        auto it = states_.getNewestWithCheck();
        return {ros::Time(it->first), it->second};
    }

    /**
     * If there are two or more states in the history: returns the previous state of the filter.
     * If there is just one state in the history: returns the current state of the filter.
     * @throws std::range_error if the filter is not yet initialized.
     */
    std::pair<ros::Time, const FilterStateWithCovariance&> getPreviousState() const
    {
        auto it = states_.getNewestWithCheck();
        if (states_.size() > 1)
        {
            --it;
        }
        return {ros::Time(it->first), it->second};
    }

    void invalidateVisualOffsetsSince(const ros::Time& ts)
    {
        const Scalar cov_growth = 10.0;                                // TODO estimate
        const Matrix<3, 3> P_pvw = Matrix<3, 3>::Identity() * cov_growth;
        const Matrix<4, 4> P_qvw = Matrix<4, 4>::Identity() * cov_growth;

        auto state_it = states_.findBefore(ts.toSec());

        state_it->second.getCovariance().block<3, 3>(StateVector::Idx::pvwx, StateVector::Idx::pvwx) += P_pvw;
        state_it->second.getCovariance().block<4, 4>(StateVector::Idx::qvww, StateVector::Idx::qvww) += P_qvw;

        while (true)
        {
            if (state_it == std::end(states_))
            {
                break;
            }
            const auto prev_it = state_it++;
            if (state_it == std::end(states_))
            {
                break;
            }

            state_it->   second.getCovariance().block<3, 3>(StateVector::Idx::pvwx, StateVector::Idx::pvwx) =
                prev_it->second.getCovariance().block<3, 3>(StateVector::Idx::pvwx, StateVector::Idx::pvwx) + P_pvw;

            state_it->   second.getCovariance().block<4, 4>(StateVector::Idx::qvww, StateVector::Idx::qvww) =
                prev_it->second.getCovariance().block<4, 4>(StateVector::Idx::qvww, StateVector::Idx::qvww) + P_qvw;
        }
    }

    /**
     * This method accepts all measurements supported by the filter.
     */
    void update(const std::shared_ptr<Measurement>& meas)
    {
        enforce("Not initialized", !states_.empty());

        const Scalar ts = meas->timestamp.toSec();

        /*
         * If this measurement is older than the oldest state, we can't fuse it, because it would require to
         * wipe out the state history entirely. Note that this also automatically handles the special case of
         * initialization.
         */
        if (states_.checkAvailability(ts) == decltype(states_)::Availability::TooOld)
        {
            ROS_WARN_THROTTLE(1, "Can't fuse the measurement - too old; type=%s", typeid(*meas).name());
            return;
        }

        /*
         * Expand the measurement set and cleanup the oldest measurements
         */
        measurements_.add(ts, meas);
        measurements_.removeOlderThanInclusive(ts - HistoryLengthSec);

        /*
         * Discard the states up to the point of the new measurement and cleanup the oldest states
         */
        states_.removeNewerThanInclusive(ts);
        states_.removeOlderThanInclusive(ts - HistoryLengthSec);

        /*
         * Now we have:
         *
         *         oldest          meas       present moment
         * meases: x x x x x x x x x x x x x x x x x x x x x
         * states: x x x x x x x x x x    (rest are removed)
         *
         * Recompute the state using all the measurements available up to the current point
         *
         * TODO: we could apply IEKF to the future states.
         */
        ROS_ASSERT(!states_.empty());
        ROS_ASSERT(!measurements_.empty());

        auto meas_it = measurements_.findAfter(states_.getNewestWithCheck()->first);  // This is the starting point
        enforce("Logic error: the newest state is newer than the newest measurement",
                meas_it != std::end(measurements_));

        {
            const int dist = static_cast<int>(std::distance(meas_it, std::end(measurements_)));
            ROS_WARN_COND(dist > 100, "Rewinding %d states", dist);
        }

        for (; meas_it != std::end(measurements_); ++meas_it)
        {
            // Extract the latest state, compute the dt relative to the latest measurement
            const auto ts_state = states_.getNewestWithCheck();
            const Scalar dt = meas_it->first - ts_state->first;
            enforce("Logic error: negative dt", dt >= 0.0);      // Technically, zero dt is possible, albeit unlikely

            // Propagate the state dt seconds in the future
            auto state = ts_state->second;
            state.propagate(dt, computeQ(dt));

            // Apply the measurement
            updateImpl(state, meas_it->second);

            // Voila - a brand new state is ready. It can be propagated further on the next iteration.
            states_.add(meas_it->first, state);
        }
    }

    /**
     * Computes how many filter updates have happened since the given time instant.
     */
    int computeHowManyUpdatesHappenedSince(const ros::Time& ts) const
    {
        return static_cast<int>(std::distance(measurements_.findMatchingOrAfter(ts.toSec()),
                                              std::end(measurements_)));
    }
};

}
