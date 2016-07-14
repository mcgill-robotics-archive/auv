#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Acoustic servo action."""

import rospy
from math import radians
from std_msgs.msg import Float32
from actionlib import SimpleActionClient
from auv_msgs.msg import SetVelocityGoal
from auv_msgs.msg import SetVelocityAction

__author__ = "Malcolm Watt"


class AcousticServo(object):
    """Acoustic servo action.

    This action tracks the difference between the current IMU heading and
    the heading required by Hydrophones. It then attempts to minimize the
    difference between the two while moving forward.
    """

    VELOCITY = rospy.get_param("taskr/velocity", default=1)
    RATE = rospy.get_param("taskr/vel_cmd_rate", default=10)
    VEL_COEFFICIENT = rospy.get_param("taskr/vel_coefficient", default=1)
    DEPTH = 2.0
    PREEMPT_CHECK_FREQUENCY = 10  # Hz

    def __init__(self, topic):
        """Constructor for the AcousticServo action.i

        Args:
            topic:  the topic name for the action
        """
        self.topic = topic

        # Keep track of the last ten orrientations for IMU and Hydrophones
        self.imu_heading_log = []
        self.imu_heading = 0
        self.pinger_heading_log = []
        self.pinger_heading = 0

        # Create velocity action client for controls server.
        self.vel_client = SimpleActionClient("controls_velocity",
                                             SetVelocityAction)
        self.vel_client.wait_for_server()

        self.server = None
        self.feedback_msg = None

        rospy.Subscriber("hyd_test", Float32, self.proc_estim_head)
        rospy.Subscriber("imu_test", Float32, self.proc_imu_head)

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
            rate.sleep()

    def proc_imu_head(self, data):
        """Update the current heading of the robot.

        Args:
            data: ROS message data object containing the current heading of the
                  robot in degrees.
        """
        self.imu_heading = data.data
        self.imu_heading_log.append(self.imu_heading)

    def proc_estim_head(self, data):
        """Update the estimated pinger heading, and send control commands.

        Args:
            data: ROS message data object containing the estimated heading of
                  the pinger in 8ths of a circle (i.e. a value between 0 & 7).
        """
        # Input range is 0-7, so to conv to degrees multiply by 45
        self.pinger_heading = data.data * 45
        self.pinger_heading_log.append(self.pinger_heading)

        imu_log = self.imu_heading_log  # Clone the log to local variable
        self.imu_heading_log = []       # Clear the log to avoid reusing data

        heading = imu_log[-1]  # The most recent imu value (units: degrees)

        target_heading = self.pinger_heading - heading % 365  # degrees
        self.move(target_heading)

    def move(self, yaw):
        """Send command to move the robot forward at the given yaw.

        Args:
            yaw: The heading we need to adjust to in degrees relative to the
                 horizon's frame of reference.
        """
        rospy.loginfo("Move at yaw: {0}".format(yaw))
        ctrl_goal = SetVelocityGoal()
        ctrl_goal.cmd.depth = self.DEPTH

        # Threshold above which we should only attempt to adjust yaw
        if yaw >= 45:
            ctrl_goal.cmd.yaw = radians(yaw)
            self.vel_client.send_goal(ctrl_goal)

        # Adjust yaw while sending one seconds worth of surge commands
        else:
            ctrl_goal.cmd.yaw = radians(yaw)

            ctrl_goal.cmd.surgeSpeed = self.VELOCITY * self.VEL_COEFFICIENT
            rate = rospy.Rate(self.RATE)

            # Send surge commands for 1 second
            for i in range(0, self.RATE):
                self.vel_client.send_goal(ctrl_goal)
                rate.sleep()
