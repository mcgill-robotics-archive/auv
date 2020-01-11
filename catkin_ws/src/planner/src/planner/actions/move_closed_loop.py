#!/usr/bin/env python

import actionlib
import math
import rospy

from planner.msg import Move_Closed_LoopAction, Move_Closed_LoopFeedback, Move_Closed_LoopResult

class Move_Closed_Loop_Server(object):

	def __init__(self,depth_topic,yaw_topic,xy_topic):
		#Set up the feedback and results variables
		self._feedback = Move_Closed_LoopFeedback()
		self._result   = Move_Closed_LoopResult()

		#Set up the Topics we're going to listen to
		self.depthsub  = rospy.Subscriber(depth_topic , Float64 , )


		#Set up the action server itself
		self.server = actionlib.SimpleActionServer(
			self._name, Move_Closed_LoopAction, self.execute, False)

    def execute(self, goal):

    def wait_until_stable(goal,target_topic):
    	


if __name__ == '__main__':
	rospy.init_node(NavigateServer.NAME)

	add_wp_topic = rospy.get_param('~/add_wp_topic', '/control/push_waypoint')
	clr_wp_topic = rospy.get_param('~/clr_wp_topic', '/control/clear_waypoints')

	server = NavigateServer(add_wp_topic, clr_wp_topic)
	rospy.spin()
