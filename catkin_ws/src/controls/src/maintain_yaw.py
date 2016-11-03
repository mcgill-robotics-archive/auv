#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Wrench


def callback():
    thrust_pub = rospy.Publisher('controls/update', Wrench)

if __name__ == '__main__':
    rospy.init_node('maintain_yaw')
    sub = rospy.Subscriber('robot_state', callback)
    rospy.spin()
