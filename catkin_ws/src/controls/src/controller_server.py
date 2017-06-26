#! /usr/bin/env python

import rospy
import actionlib

from controls.msg import GoToDepthAction
from controller import DepthMaintainer


class GoToDepthServer(object):
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            'go_to_depth',
            GoToDepthAction,
            self.execute)

    def execute(self, goal):
        depth_maintainer = DepthMaintainer(goal.depth)
        period = rospy.Duration(0, 10**8)  # 10**8 nsecs = 0.1 secs
        depth_maintainer.start(period)

        cnt = 0
        while cnt < goal.stable_periods:
            if abs(depth_maintainer.error[0]) < goal.stable_threshold:
                cnt += 1
            else:
                cnt = 0
            rospy.sleep(1)

        depth_maintainer.stop()
        self.server.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('go_to_depth_server')
    server = GoToDepthServer()
    rospy.spin()
