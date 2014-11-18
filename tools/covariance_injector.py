#!/usr/bin/env python
#
# Copyright (c) 2014 Zubax, zubax.com
# Please refer to the file LICENSE for terms and conditions.
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#
# This script overrides uncertainty estimates on IMU and GNSS topics.
# Use it like this:
#     ./covariance_injector.py imu_in:=/imu gnss_in:=/gps imu_out:=/dev/imu/data gnss_out:=/dev/gnss/fix
#

import roslib
roslib.load_manifest('zubax_posekf')

import rospy
from functools import partial
from sensor_msgs.msg import Imu
from gps_common.msg import GPSFix


def interpolate(x, x_range, y_range):
    yl, yh = y_range
    xl, xh = x_range
    return yl + (yh - yl) * float(x - xl) / (xh - xl)


def cb_imu(msg):
    msg.linear_acceleration_covariance = [0.0 for _ in range(9)]
    msg.linear_acceleration_covariance[0] = accel_variance
    msg.linear_acceleration_covariance[4] = accel_variance
    msg.linear_acceleration_covariance[8] = accel_variance

    msg.angular_velocity_covariance = [0.0 for _ in range(9)]
    msg.angular_velocity_covariance[0] = gyro_variance
    msg.angular_velocity_covariance[4] = gyro_variance
    msg.angular_velocity_covariance[8] = gyro_variance

    pub_imu.publish(msg)


def cb_gnss(msg):
    interp = partial(interpolate, msg.status.satellites_used, gnss_sat_range)

    msg.err_horz  = interp(gnss_err_horz)
    msg.err_vert  = interp(gnss_err_vert)
    msg.err_track = interp(gnss_err_track)
    msg.err_speed = interp(gnss_err_speed)
    msg.err_climb = interp(gnss_err_climb)

    if msg.position_covariance_type == msg.COVARIANCE_TYPE_UNKNOWN:
        msg.position_covariance = [0.0 for _ in range(9)]
        msg.position_covariance[0] = msg.err_horz ** 2
        msg.position_covariance[4] = msg.err_horz ** 2
        msg.position_covariance[8] = msg.err_vert ** 2

    pub_gnss.publish(msg)


rospy.init_node('covariance_injector')

accel_variance = rospy.get_param('~accel_variance', 0.5)
gyro_variance = rospy.get_param('~gyro_variance', 0.01)

gnss_sat_range = rospy.get_param('~gnss_sat_range', [11, 6])
gnss_err_horz  = rospy.get_param('~gnss_err_horz',  [3, 30])
gnss_err_vert  = rospy.get_param('~gnss_err_vert',  [7, 50])
gnss_err_track = rospy.get_param('~gnss_err_track', [1, 40])
gnss_err_speed = rospy.get_param('~gnss_err_speed', [0.1, 3])
gnss_err_climb = rospy.get_param('~gnss_err_climb', [0.5, 4])

sub_imu  = rospy.Subscriber('imu_in', Imu, cb_imu)
sub_gnss = rospy.Subscriber('gnss_in', GPSFix, cb_gnss)

pub_imu  = rospy.Publisher('imu_out', Imu, queue_size=10)
pub_gnss = rospy.Publisher('gnss_out', GPSFix, queue_size=10)

rospy.spin()
