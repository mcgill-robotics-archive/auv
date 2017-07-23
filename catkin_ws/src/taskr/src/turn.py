#!/usr/bin/env python
import rospy
from controls.servo_controller import YawMaintainer, DepthMaintainer


class Turn(object):
    def __init__(self, data):
        self.preempted = False
        self.angle = data["angle"]

    def start(self, server, feedback_msg):
        rospy.loginfo("Starting turn action")

        yaw_maintainer = YawMaintainer(self.angle)
        yaw_maintainer.start()
        depth_maintainer = DepthMaintainer()
        depth_maintainer.start()

        depth_maintainer = DepthMaintainer()
        depth_maintainer.start()

        stable_counts = 0
        while stable_counts < 30:
            rospy.loginfo("{} / 30 stable periods achieved".format(
                stable_counts))

            if self.preempted:
                yaw_maintainer.stop()
                depth_maintainer.stop()
                return

            err = yaw_maintainer.get_error()
            if abs(err) < 0.1:
                stable_counts += 1
            else:
                stable_counts = 0
            rospy.sleep(0.1)

        yaw_maintainer.stop()
        depth_maintainer.stop()
        rospy.loginfo("Done turn acion")

    def stop(self):
        self.preempted = True
