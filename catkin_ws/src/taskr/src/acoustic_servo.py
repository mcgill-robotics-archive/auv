#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Acoustic servo action."""

import rospy
from move import Move
from math import fabs
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped

__author__ = "Malcolm Watt and Wei-Di Chang"


class AcousticServo(object):
    """Acoustic servo action.

    This action tracks the difference between the current IMU heading and
    the Hydrophones determined heading. It then attempts to move towards
    the pinger.
    """
    SURGE_STEP = 0.7
    PREEMPT_CHECK_FREQUENCY = 1  # Hz
    MAX_AGE = 6  # Seconds
    TIMEOUT = 300  # Seconds = 5 min

    def __init__(self, config):
        """Constructor for the AcousticServo action.

        Args:
            topic:  the topic name for the action
        """
        self.depth = config["depth"] if "depth" in config else None

        # Keep track of current IMU and last 10 Hydrophones heading
        self.robot_heading = None
        self.heading = None
        self.last_heading = None

        self.pinger_heading_log = []
        self.pinger_heading = 0

        # Keep track of ping timestamps
        self.heading_time = rospy.Time.now()
        self.last_heading_time = 0

        self.heading_error = 0

        self.server = None
        self.feedback_msg = None

        self.move_cmd = {"distance": self.SURGE_STEP,
                         "yaw": self.heading,
                         "feedback": False}

        self.start_time = rospy.Time.now()

        rospy.Subscriber("hydrophones/heading", Float64, self.proc_estim_head)
        self.pose_sub = rospy.Subscriber('robot_state', Vector3Stamped, self.state_cb)
        self.depth_sub = rospy.Subscriber('state_estimation/depth', Float64, self.depth_callback)

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

        while self.depth is None:
            rospy.loginfo("AS: No depth was provided, waiting for current depth.")
            rate.sleep()

        while not rospy.is_shutdown():
            # Check for preempt
            if server.is_preempt_requested():
                rospy.loginfo("AcousticServo preempted")
                server.set_preempted()
                return

            # We have not yet received a heading. Do not move.
            if self.heading is None:
                rospy.loginfo("No pinger command. Staying still and waiting.")
                move_cmd = {"distance": 0,
                            "depth": self.depth}
                move_action = Move(move_cmd)
                move_action.start(self.server, self.feedback_msg)
                continue

            move_cmd = {"distance": self.SURGE_STEP,
                        "yaw": self.heading,
                        "depth": self.depth}

            # If we don't have a last heading, we can't determine if done.
            if self.last_heading is not None:
                # If angle is greater than 90, we have reached the pinger.
                if fabs(self.heading - self.last_heading) > 1.57:
                    rospy.loginfo("Pinger has been reached! Ending")
                    break

            if (rospy.Time.now() - self.heading_time).to_sec() < self.MAX_AGE:
                rospy.loginfo("Sending move command towards pinger {}".format(move_cmd))
                move_action = Move(move_cmd)
                move_action.start(self.server, self.feedback_msg)
            else:
                rospy.loginfo("Pinger message is too old. Sending command {}".format(move_cmd))
                move_cmd = {"distance": 0,
                            "depth": self.depth}
                move_action = Move(move_cmd)
                move_action.start(self.server, self.feedback_msg)

            if (rospy.Time.now() - self.start_time).to_sec() > self.TIMEOUT:
                rospy.loginfo("Acoustic servo has timed out.")
                return

            rate.sleep()

        if server.is_preempt_requested():
            rospy.loginfo("AcousticServo preempted")
            server.set_preempted()
            return

    def state_cb(self, msg):
        self.robot_heading = msg.vector.z

    def proc_estim_head(self, msg):
        """Update the estimated pinger heading, and send control commands.

        Args:
            data: ROS message data object containing the estimated heading of
                  the pinger (i.e. -pi to pi).
        """
        if self.robot_heading is None:
            rospy.logerr("No robot heading. Panic.")
            return

        self.last_heading = self.heading
        self.last_heading_time = self.heading_time

        self.heading_time = rospy.Time.now()
        self.pinger_heading = msg.data
        self.pinger_heading_log.append(self.pinger_heading)

        self.heading = self.robot_heading + self.pinger_heading
        self.heading_error = self.robot_heading - self.pinger_heading

    def depth_callback(self, msg):
        self.depth = msg.data
