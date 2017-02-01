#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped


def pub_yaw_cb(event):
    yaw = Vector3Stamped()
    yaw.vector.z = 1.0
    pub_yaw.publish(yaw)


def pub_hydro_cb(event):
    hydro = Float64()
    hydro.data = 2.0
    pub_hydro.publish(hydro)


if __name__ == '__main__':
    rospy.init_node("hydro_test")

    pub_yaw = rospy.Publisher("robot_state", Vector3Stamped, queue_size=1)
    pub_hydro = rospy.Publisher("hydrophones/heading", Float64, queue_size=1)

    rospy.Timer(rospy.Duration(0.1), pub_yaw_cb)
    rospy.Timer(rospy.Duration(2), pub_hydro_cb)

    rospy.spin()
