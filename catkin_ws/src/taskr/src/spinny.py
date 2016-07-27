#!/usr/bin/env python
import rospy
from math import fabs
from actionlib import SimpleActionClient
from geometry_msgs.msg import Vector3Stamped
from auv_msgs.msg import SetVelocityAction, SetVelocityGoal
from std_msgs.msg import Float64


class Spinny(object):
    """Spin action."""

    RATE = rospy.get_param("taskr/vel_cmd_rate", default=10)
    ANGLE_INCREMENT = 0.1

    def __init__(self, point):
        """Constructor for the Spinny object."""
        self.curr_yaw = 0
        self.curr_depth = 0

        self.depth_sub = rospy.Subscriber("state_estimation/depth", Float64, self.depth_callback)
        self.pose_sub = rospy.Subscriber("robot_state", Vector3Stamped, self.pose_callback)

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

        # If depth was not set, take the current depth.
        while not self.curr_depth:
            pass

        ctrl_goal.cmd.depth = self.curr_depth

        # If yaw was not set, take the current yaw.
        while not self.curr_yaw:
            pass

        start_yaw = self.curr_yaw

        ctrl_goal.cmd.yaw = start_yaw + self.ANGLE_INCREMENT
        self.vel_client.send_goal(ctrl_goal)
        self.vel_client.wait_for_result()

        while not fabs(self.curr_yaw - start_yaw) < 0.001:
            ctrl_goal.cmd.yaw += self.ANGLE_INCREMENT
            self.vel_client.send_goal(ctrl_goal)

            # Check if we received preempt request from Planner
            if server.is_preempt_requested():
                rospy.logerr("Spinny preempted")
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

        rospy.loginfo("Done spiny in time {}".format((rospy.Time.now() - start).to_sec()))

    def depth_callback(self, msg):
        self.curr_depth = msg.data

    def pose_callback(self, msg):
        self.curr_yaw = msg.vector.z
