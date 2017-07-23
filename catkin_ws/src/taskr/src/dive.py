#!/usr/bin/env python
import rospy
<<<<<<< HEAD
from math import fabs
from std_msgs.msg import Float64
from controls.servo_controller import DepthMaintainer
=======
from controls.servo_controller import YawMaintainer, DepthMaintainer
>>>>>>> dev


class Dive(object):
    '''
    Dive is an action that will cause the robot to dive to and stabilize at a
    given depth.
    '''
    def __init__(self, data):
        self.preempted = False
        self.depth = data["depth"]

    def start(self, server, feedback_msg):
        rospy.loginfo("Starting dive action")

        yaw_maintainer = YawMaintainer()
        yaw_maintainer.start()

        depth_maintainer = DepthMaintainer(self.depth)
        depth_maintainer.start()

        stable_counts = 0
        while stable_counts < 30:
            rospy.loginfo("{} / 30 stable periods achieved".format(
                stable_counts))

            if self.preempted:
                depth_maintainer.stop()
                yaw_maintainer.stop()
                return

            err = depth_maintainer.error
            if err is None:
                pass
            elif abs(err) < 0.1:
                stable_counts += 1
            else:
                stable_counts = 0
            rospy.sleep(0.1)

        depth_maintainer.stop()
        yaw_maintainer.stop()
        rospy.loginfo("Done dive acion")

    def stop(self):
        self.preempted = True
