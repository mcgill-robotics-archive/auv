#!/usr/bin/env python

import rospy
from actionlib import *
from actionlib_msgs.msg import *
from planner.msg import *
from geometry_msgs.msg import *
from auv_msgs.msg import SetVelocityAction, SetVelocityFeedback, SetVelocityResult, SetVelocity
from actionlib import SimpleActionServer, SimpleActionClient

class Taskr(object):
    _feedback = moveFeedback()
    _result = moveResult()

    def __init__(self, name):
        # Create move action server
        self._action_name = name
        self._as = SimpleActionServer(self._action_name, moveAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        # Create velocity action client
        vel_client = SimpleActionClient('controls_velocity', SetVelocityAction)

        time = int(goal.time.secs)
        velocity = goal.velocity
        theta = goal.theta
        depth = goal.depth

        ctrl_goal = SetVelocity()

        if theta != 0:
            # Turn first before surging
            ctrl_goal.yaw = theta
            # Send to velocity server
            vel_client.send_goal(ctrl_goal)
            vel_client.wait_for_result()

        # Surge
        rate = rospy.Rate(1)
        for s in range(1, time, 1):
            ctrl_goal.surgeSpeed = velocity
            vel_client.send_goal(ctrl_goal)
            # Sleep for the amount of time that makes the for-loop run for 1 second
            rate.sleep()
        self._result.success = True
        self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('taskr')
    Taskr('square_action')
    rospy.spin()
