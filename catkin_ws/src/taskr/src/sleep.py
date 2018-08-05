#!/usr/bin/env python
import rospy
from math import fabs
from std_msgs.msg import Float64

class Sleep(object):
    """Move action."""
    SLEEP_TIME = rospy.get_param("taskr/sleep_time", 15)

    def __init__(self, point):
        self.preempted = False
    def start(self, server, feedback_msg):


        for i in range(0, int(10 * self.SLEEP_TIME)):
            if self.preempted:
                return
            rospy.loginfo(
                "Sending cmd {}s / {}s".format(i, int(10 * self.SLEEP_TIME)))
            rospy.sleep(0.1)

    def stop(self):
        self.preempted = True
