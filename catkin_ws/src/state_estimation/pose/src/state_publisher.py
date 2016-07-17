#!/usr/bin/env python
import numpy as np
import rospy
import tf
from geometry_msgs.msg import Vector3Stamped
from tf.transformations import euler_from_quaternion

listener = None

def get_robot_pose():
    global listener
    listener.waitForTransform("/initial_horizon", "/robot", rospy.Time(), rospy.Duration(0.1))

    (trans, rot) = listener.lookupTransform(
        # FROM
        "/initial_horizon",
        # TO
        "/robot",
        # NOW
        rospy.Time())
    return (trans, rot)

if __name__ == '__main__':
    rospy.init_node('state_publisher')
    listener = tf.TransformListener()
    state_pub = rospy.Publisher('robot_state',
        Vector3Stamped,
        queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
    	(trans, rot) = get_robot_pose()
    	(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)
    	msg = Vector3Stamped()
    	msg.header.stamp = rospy.Time()
    	msg.vector.x = roll
    	msg.vector.y = pitch
    	msg.vector.z = yaw
    	state_pub.publish(msg)
    	rate.sleep()

 
