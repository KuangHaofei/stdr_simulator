#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
import tf

from tf import transformations
import numpy as np
from math import *

from geometry_msgs.msg import PoseStamped


def get_pose_tf(msg):
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    target_frame = 'map'

    transform = tf_buffer.lookup_transform(target_frame,
                                           msg.header.frame_id,
                                           rospy.Time(0),
                                           rospy.Duration(1))

    pose = tf2_geometry_msgs.do_transform_pose(msg, transform)

    return pose


def get_pose_numpy(msg):
    # get position data
    pose = PoseStamped()
    x = -7.75
    y = -7.46

    p = np.array([msg.pose.position.x, msg.pose.position.y, 1])

    # define transform matrix
    theta = 0
    T = np.array([[cos(theta),  -sin(theta), x],
                  [sin(theta),  cos(theta),  y],
                  [0,           0,           1]])

    # compute pose
    pose.header = msg.header
    pose.header.frame_id = 'map'

    point = T.dot(p)

    pose.pose = msg.pose
    pose.pose.position.x = point[0]
    pose.pose.position.y = point[1]

    return pose


def callback(msg):
    pose = get_pose_numpy(msg)

    pub.publish(pose)


if __name__ == '__main__':
    rospy.init_node('map_pose')

    pub = rospy.Publisher('/slam_out_pose', PoseStamped, queue_size=1)

    rospy.Subscriber('/pose', PoseStamped, callback)

    rospy.spin()