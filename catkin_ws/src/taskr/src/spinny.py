#!/usr/bin/env python
import rospy
import numpy
from math import fabs, pi
from actionlib import SimpleActionClient
from geometry_msgs.msg import Point, Vector3Stamped
from auv_msgs.msg import SetVelocityAction, SetVelocityGoal
from std_msgs.msg import Float64


class Move(object):
    """Move action."""

    RATE = rospy.get_param("taskr/vel_cmd_rate", default=10)
    ANGLE_INCREMENT = 0.1

    def __init__(self, point):
        """Constructor for the Spinny object."""
        self.curr_yaw = 0
        self.curr_depth = 0

        self.depth_sub = rospy.Subscriber('state_estimation/depth', Float64, self.depth_callback)
        self.pose_sub = rospy.Subscriber('robot_state', Vector3Stamped, self.pose_callback)

        self.depth = point["depth"]
        self.yaw = point["yaw"]
        self.sway = point["sway"] if "sway" in point else False
        self.maintain_yaw = (self.yaw == "same")
        self.maintain_depth = (self.depth == "same")

        # Create velocity action client for controls server.
        self.vel_client = SimpleActionClient("controls_velocity", SetVelocityAction)
        self.vel_client.wait_for_server()        

    def start(self, server, feedback_msg):
        """Do the spin action.

        Args:
            server: Action server for publishing feedback.
            feedback_msg: Feedback message to populate.
        """
        rate = rospy.Rate(self.RATE)

        ctrl_goal = SetVelocityGoal()

        if self.maintain_depth:
            ctrl_goal.cmd.depth = self.curr_depth
        else:
            ctrl_goal.cmd.depth = self.depth


        for yaw in range(self.curr_yaw, self.curr_yaw + 2*pi, ANGLE_INCREMENT):
            ctrl_goal.cmd.yaw = yaw
            self.vel_client.send_goal(ctrl_goal)

            # Check if we received preempt request from Planner
            if server.is_preempt_requested():
                rospy.logerr("Taskr preempted")
                # Send preempt request to Controls
                self.vel_client.cancel_goal()
                server.set_preempted()
                return

            feedback_msg.is_done = False  # Not super useful feedback.
            server.publish_feedback(feedback_msg)
            # self.vel_client.wait_for_result()

            rate.sleep()

        self.vel_client.wait_for_result()

        start = rospy.Time.now()

        rospy.loginfo("Done move in time {}".format((rospy.Time.now() - start).to_sec()))

    def depth_callback(self, msg):
        self.curr_depth = msg.data

    def pose_callback(self, msg):
        self.curr_yaw = msg.vector.z
