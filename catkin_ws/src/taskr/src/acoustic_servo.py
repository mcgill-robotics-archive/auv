#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Acoustic servo action."""

import rospy
import tf
from math import radians
from std_msgs.msg import Float32
from actionlib import SimpleActionClient
from auv_msgs.msg import SetVelocityGoal, SetVelocityAction
from geometry_msgs.msg import Vector3Stamped

# from utils import get_yaw_and_depth
from move import Move

__author__ = "Malcolm Watt and Wei-Di Chang"

class AcousticServo(object):
    """Acoustic servo action.

    This action tracks the difference between the current IMU heading and
    the Hydrophones determined heading. It then attempts to move towards
    the pinger.
    """
    DEPTH = 0.0
    SURGE_STEP = 0.5
    PREEMPT_CHECK_FREQUENCY = 10  # Hz    



    def __init__(self, topic):
        """Constructor for the AcousticServo action.

        Args:
            topic:  the topic name for the action
        """
        self.topic = topic

        # Keep track of current IMU and last 10 Hydrophones heading
        self.robot_heading = 0

        self.pinger_heading_log = []
        self.pinger_heading = 0

        self.heading_error = 0

        self.server = None
        self.feedback_msg = None

        rospy.Subscriber("hyd_test", Float32, self.proc_estim_head)
        self.pose_sub = rospy.Subscriber('robot_state', Vector3Stamped, self.state_cb)

    def start(self, server, feedback_msg):
        """Servo toward the pinger.

        Args:
            server:         Action server for publishing feedback.
            feedback_msg:   Feedback message to populate.
        """
        rospy.loginfo("Starting AcousticServo Action")

        self.server = server
        self.feedback_msg = feedback_msg

        rate = rospy.Rate(self.PREEMPT_CHECK_FREQUENCY)

        while True:
            if server.is_preempt_requested():
                rospy.loginfo("AcousticServo preempted")
                server.set_preempted()
                return
            rospy.sleep(0.1)
            continue

    def state_cb(self, msg):
        self.robot_heading = msg.vector.z

    def proc_estim_head(self, msg):
        """Update the estimated pinger heading, and send control commands.

        Args:
            data: ROS message data object containing the estimated heading of
                  the pinger (i.e. -pi to pi).
        """
        # Input range is -PI-PI
        self.pinger_heading = msg.data
        self.pinger_heading_log.append(self.pinger_heading)

        self.heading_error = self.robot_heading - self.pinger_heading   

        print "DDDD"

        move_cmd = {"distance": self.SURGE_STEP,
                     "depth": self.DEPTH,
                     "yaw": self.pinger_heading,
                     "feedback": False}
        move_action = Move(move_cmd)
        move_action.start(self.server, self.feedback_msg)


