#!/usr/bin/env python
import rospy
from controls.maintainers import YawMaintainer, DepthMaintainer, RollMaintainer

class Roll(object):
    def __init__(self, data):
        self.preempted = False
        self.angle = data["angle"]

        self.ROLL_DURATION = rospy.get_param("taskr/roll/roll_duration",30)
        self.depth_maintainer = DepthMaintainer(data["depth"])
        self.yaw_maintainer = YawMaintainer()
        self.roll_maintainer = RollMaintainer(self.angle)

    def start(self, server, feedback_msg):
        rospy.loginfo("Starting roll action")
        if not self.depth_maintainer.is_active():
            self.depth_maintainer.start()

        if not self.yaw_maintainer.is_active():
            self.yaw_maintainer.start()

        if not self.roll_maintainer.is_active():
            self.roll_maintainer.start()

        duration_counts = 0
        while duration_counts < self.ROLL_DURATION:
            rospy.loginfo("{} / {}  roll periods achieved".format(
                duration_counts,self.ROLL_DURATION))

            if self.preempted:
                return

            duration_counts += 1
            rospy.sleep(0.1)

        rospy.loginfo("Done roll acion")

    def stop(self):
        self.preempted = True
        if self.depth_maintainer.is_active():
            self.depth_maintainer.stop()
        if self.yaw_maintainer.is_active():
            self.yaw_maintainer.stop()
        if self.roll_maintainer.is_active():
            self.roll_maintainer.stop()
