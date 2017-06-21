#! /usr/bin/env python


import rospy
import actionlib

from controls.msg import GoToDepthAction, GoToDepthGoal


if __name__ == '__main__':
    rospy.init_node('test_depth_action')
    client = actionlib.SimpleActionClient('go_to_depth', GoToDepthAction)
    client.wait_for_server()

    # TODO: All testing values. Replace these with params
    goal = GoToDepthGoal()
    goal.depth = 3.0
    goal.stable_threshold = 0.3
    goal.stable_periods = 20
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(30))
