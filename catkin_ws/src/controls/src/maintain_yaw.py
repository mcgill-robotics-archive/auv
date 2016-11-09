#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Wrench, Vector3
from tf import TransformListener


class YawMaintainer():
    def __init__(self, desired_yaw):
        self.listener = TransformListener()
        self.thrust_pub = rospy.Publisher('controls/update', Wrench)
        self.desired_yaw = desired_yaw

        self.set_yaw(desired_yaw)

    def set_yaw(self, yaw):
        pass

    def update(self):
        pass


if __name__ == '__main__':
    rospy.init_node('maintain_depth')
    # TODO: figure out how to initialize the yaw desired
    yaw_maintainer = YawMaintainer(1)
    rospy.spin()
