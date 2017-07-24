#!/usr/bin/env python
import rospy
from controls.servo_controller import YawMaintainer, DepthMaintainer


class Dive(object):
    '''
    Dive is an action that will cause the robot to dive to and stabilize at a
    given depth.
    '''
    def __init__(self, data):
        self.preempted = False
        self.depth = data["depth"]
        self.yaw_maintainer = YawMaintainer()
        self.depth_maintainer = DepthMaintainer(self.depth)

    def start(self, server, feedback_msg):
        rospy.loginfo("Starting dive action")

        if not self.yaw_maintainer.is_active():
            self.yaw_maintainer.start()
        if not self.depth_maintainer.is_active():
            self.depth_maintainer.start()

        stable_counts = 0
        while stable_counts < 30:
            rospy.loginfo("{} / 30 stable periods achieved".format(
                stable_counts))

            if self.preempted:
                return

            err = self.depth_maintainer.error
            if err is None:
                pass
            elif abs(err) < 0.1:
                stable_counts += 1
            else:
                stable_counts = 0
            rospy.sleep(0.1)

        rospy.loginfo("Done dive acion")

    def stop(self):
        self.preempted = True

        if self.depth_maintainer.is_active():
            self.depth_maintainer.stop()
        if self.yaw_maintainer.is_active():
            self.yaw_maintainer.stop()
