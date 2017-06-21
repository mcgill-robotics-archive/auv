#! /usr/bin/env python

import rospy
import actionlib
from std_msgs.msg import Float64

from taskr.msg import SurgeAction
from controls.depth_maintainer import DepthMaintainer
from controls.yaw_maintainer import YawMaintainer


class SurgeServer(object):
    def __init__(self):
        self.server = actionlib.SimpleActionServer('surge', SurgeAction,
                                                   self.execute, False)
        self.surge_pub = rospy.Publisher('controls/superimposer/surge',
                                         Float64)
        self.depth_maintainer = DepthMaintainer().start()
        self.yaw_maintainer = YawMaintainer().start()
        self.server.start()

    def execute(self, goal):
        rate = rospy.Rate(goal.freq)
        self.depth_maintainer.start()
        self.yaw_maintainer.start()

        for i in range(0, goal.duration/goal.freq):
            self.surge_pub.publish(goal.thrust)
            rate.sleep()

        self.depth_maintainer.end()
        self.yaw_maintainer.end()
        self.server.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('surge_server')
    server = SurgeServer()
    rospy.spin()
