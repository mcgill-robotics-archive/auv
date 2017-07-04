#!/usr/bin/env python
import rospy
import tf
import math


def get_current_depth():
    '''
    Handles the transform lookup to get the current depth
    '''
    listener = tf.TransformListener()
    listener.waitForTransform('robot', 'floating_horizon', rospy.Time(), rospy.Duration(2.0))

    try:
        now = rospy.Time.now()
        listener.waitForTransform('robot', 'floating_horizon', now, rospy.Duration(2.0))
        trans, rot = listener.lookupTransform('robot', 'floating_horizon', now)
        return trans[2]
    except Exception as e:
        rospy.logerr("Couldn't get depth transform: {}".format(e))
        return 0.0


def get_current_yaw():
    listener = tf.TransformListener()
    listener.waitForTransform('robot', 'initial_horizon', rospy.Time(), rospy.Duration(2.0))

    try:
        now = rospy.Time.now()
        listener.waitForTransform('robot', 'initial_horizon', now, rospy.Duration(2.0))
        trans, rot = listener.lookupTransform('robot', 'initial_horizon', now)
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)
        return yaw
    except Exception as e:
        rospy.logerr("Couldn't get yaw transform: {}".format(e))
        return 0.0


def get_depth_error(setpoint):
    estimated_depth = get_current_depth()

    if estimated_depth is None:
        return 0

    return setpoint - estimated_depth


def normalize_angle(angle):
        angle = angle % (2 * math.pi)
        if angle > math.pi:
            angle -= 2 * math.pi
        return angle


def get_yaw_error(setpoint):
    estimated_yaw = get_current_yaw()

    if estimated_yaw is None:
        return 0

    yaw_error = setpoint - estimated_yaw
    return normalize_angle(yaw_error)
