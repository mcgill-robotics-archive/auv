#!/usr/bin/env python

import rospy
from actionlib import *
from actionlib_msgs.msg import *
from planner.msg import *
from geometry_msgs.msg import *
from auv_msgs.msg import (SetVelocityAction, SetVelocityFeedback,
    SetVelocityResult, SetVelocityGoal, VisualServoAction, VisualServoGoal,
    VisualServo)
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
        print 'Received goal'
        print goal
        # Create velocity action client
        vel_client = SimpleActionClient('controls_velocity', SetVelocityAction)
        servo_client.wait_for_server()

        time = int(goal.time.secs)
        velocity = goal.velocity
        theta = goal.theta
        depth = goal.depth

        ctrl_goal = SetVelocityGoal()
        ctrl_goal.cmd.depth = depth

        if theta != 0:
            print 'Sending Yaw'
            # Turn first before surging
            ctrl_goal.cmd.yaw = theta
            # Send to velocity server
            vel_client.send_goal(ctrl_goal)
            # Check if we received preempt request from Planner
            if self._as.is_preempt_requested():
                print "Taskr preempted"
                # Send preempt request to Controls
                vel_client.cancel_goal()
                # Handle preemption
                self._as.set_preempted()
                return
            vel_client.wait_for_result()

        # Surge
        rate = rospy.Rate(10)
        for s in range(1, time*10+1, 1):
            print 'Sending Surge'
            ctrl_goal.cmd.surgeSpeed = velocity.linear.x
            ctrl_goal.cmd.yaw = theta
            vel_client.send_goal(ctrl_goal)
            # Check if we received preempt request from Planner
            if self._as.is_preempt_requested():
                print "Taskr preempted"
                # Send preempt request to Controls
                vel_client.cancel_goal()
                # Handle preemption
                self._as.set_preempted()
                return
            # Sleep for the amount of time that makes the for-loop run for 1 second
            rate.sleep()

        self._result.success = True
        self._as.set_succeeded(self._result)
        print 'Success!'

# def test_servo():
#     # Create visual servo client
#     servo_client = SimpleActionClient('controls_vservo', VisualServoAction)
#     # Wait for visual servo server to finish starting up
#     servo_client.wait_for_server()
#     # Create goal
#     goal = VisualServoGoal()
#     goal.cmd = VisualServo()
#     goal.cmd.target_frame_id = "buoy"
#     goal.cmd.roll = 0
#     goal.cmd.pitch = 0
#     goal.cmd.yaw = 0
#     # Send goal to server
#     servo_client.send_goal(goal)
#     print "Sent visual servo goal"

# if __name__ == '__main__':
#     rospy.init_node('taskr')
#     Taskr('square_action')
#     test_servo()
#     rospy.spin()
