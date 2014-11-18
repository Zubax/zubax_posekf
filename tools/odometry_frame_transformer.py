#!/usr/bin/env python
#
# Copyright (c) 2014 Zubax, zubax.com
# Please refer to the file LICENSE for terms and conditions.
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#
# This node transforms frame identifiers in an odometry topic and then copies it into /tf
# Use it like this:
#     ./odometry_frame_transformer.py odom_in:=/odom_gt odom_out:=/ground_truth _child_frame_id:=ground_truth
#

import roslib
roslib.load_manifest('zubax_posekf')

import rospy
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster


quat_msg2arr = lambda q: (q.x, q.y, q.z, q.w)
vect_msg2arr = lambda v: (v.x, v.y, v.z)


def broadcast_transform_from_odometry(odom_msg):
    translation = vect_msg2arr(odom_msg.pose.pose.position)
    rotation = quat_msg2arr(odom_msg.pose.pose.orientation)
    tf_broadcaster.sendTransform(translation, rotation, odom_msg.header.stamp,
                                 odom_msg.child_frame_id, odom_msg.header.frame_id)


def cb_odom(msg):
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id

    pub_odom.publish(msg)
    broadcast_transform_from_odometry(msg)


rospy.init_node('odometry_frame_transformer')

frame_id = rospy.get_param('~frame_id', "world")
child_frame_id = rospy.get_param('~child_frame_id', "odom")

sub_odom = rospy.Subscriber('odom_in', Odometry, cb_odom)
pub_odom = rospy.Publisher('odom_out', Odometry, queue_size=10)

tf_broadcaster = TransformBroadcaster()

rospy.spin()
