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

import rospy, numpy, math
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from tf.transformations import translation_matrix, quaternion_matrix
from tf.transformations import quaternion_inverse, quaternion_from_euler, quaternion_multiply, euler_from_quaternion


vect_msg2arr = lambda v: numpy.array((v.x, v.y, v.z))
quat_msg2arr = lambda q: numpy.array((q.x, q.y, q.z, q.w))

def quat_arr2msg(arr, msg):
    msg.x, msg.y, msg.z, msg.w = arr

def vect_arr2msg(arr, msg):
    msg.x, msg.y, msg.z = arr


def transform_vector(vector, (translation, rotation)):
    transform = numpy.dot(translation_matrix(translation), quaternion_matrix(rotation))
    rotated = numpy.dot(transform, numpy.array([vector[0], vector[1], vector[2], 1.0]))[:3]
    return rotated


def broadcast_transform_from_odometry(odom_msg):
    translation = vect_msg2arr(odom_msg.pose.pose.position)
    rotation = quat_msg2arr(odom_msg.pose.pose.orientation)
    tf_broadcaster.sendTransform(translation, rotation, odom_msg.header.stamp,
                                 odom_msg.child_frame_id, odom_msg.header.frame_id)


def cb_odom(msg):
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id

    position = vect_msg2arr(msg.pose.pose.position)
    orientation = quat_msg2arr(msg.pose.pose.orientation)

    position = transform_vector(position, (translation_vec, rotation_quat))
    orientation = quaternion_multiply(rotation_quat, orientation)

    vect_arr2msg(position, msg.pose.pose.position)
    quat_arr2msg(orientation, msg.pose.pose.orientation)

    pub_odom.publish(msg)
    broadcast_transform_from_odometry(msg)


rospy.init_node('odometry_frame_transformer')

frame_id = rospy.get_param('~frame_id', "world")
child_frame_id = rospy.get_param('~child_frame_id', "odom")

rotation_roll  = math.radians(rospy.get_param('~rotation_roll', 0.0))
rotation_pitch = math.radians(rospy.get_param('~rotation_pitch', 0.0))
rotation_yaw   = math.radians(rospy.get_param('~rotation_yaw', 0.0))

translation_x = rospy.get_param('~translation_x', 0.0)
translation_y = rospy.get_param('~translation_y', 0.0)
translation_z = rospy.get_param('~translation_z', 0.0)

rotation_quat = quaternion_from_euler(rotation_roll, rotation_pitch, rotation_yaw)
translation_vec = numpy.array((translation_x, translation_y, translation_z))

print 'Rotation:   ', euler_from_quaternion(rotation_quat)
print 'Translation:', translation_vec

sub_odom = rospy.Subscriber('odom_in', Odometry, cb_odom)
pub_odom = rospy.Publisher('odom_out', Odometry, queue_size=10)

tf_broadcaster = TransformBroadcaster()

rospy.spin()
