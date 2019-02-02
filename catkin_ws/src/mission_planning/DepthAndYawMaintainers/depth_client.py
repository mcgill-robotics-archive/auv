#! /usr/bin/env python

from __future__ import print_function

import rospy
# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import depth.msg

def depth_client(depthGoal):
    # Creates the SimpleActionClient,
    client = actionlib.SimpleActionClient('DEPTH', depth.msg.depthAction)
    
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = depth.msg.fibsGoal(depth=depthGoal)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('depth_client_py')
        result = depth_client()
        print("Result:", ', '.join([str(n) for n in result.sequence]))
        
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
