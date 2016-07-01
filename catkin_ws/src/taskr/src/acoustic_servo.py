#!/usr/bin/env python
import rospy
from actionlib import SimpleActionClient
from auv_msgs.msg import SetVelocityAction


class AcousticServo(object):
    """Acoustic servo action.

    This action tracks the difference between the current IMU heading and
    the heading required by Hydrophones. It then attempts to minimize the
    difference between the two while moving forward, in effect moving us
    towards the pinger.
    """

    VELOCITY = rospy.get_param("taskr/velocity", default=1)
    RATE = rospy.get_param("taskr/vel_cmd_rate", default=10)
    VEL_COEFFICIENT = rospy.get_param("taskr/vel_coefficient", default=1)

    def __init__(self, topic):
        """Constructor for the AcousticServo action."""
        self.topic = topic

        # Could use a queue
        self.imu_heading = []
        self.pinger_heading = []

        # Create velocity action client for controls server.
        self.vel_client = SimpleActionClient("controls_velocity",
                                             SetVelocityAction)
        self.vel_client.wait_for_server()

    def start(self, server, feedback_msg):
        """Servo toward the pinger.

        Args:
            server: Action server for publishing feedback.
            feedback_msg: Feedback message to populate.
        """

        # @TODO Initialize the IMU subscriber, and the Hydrophones subscriber
        if server.is_preempt_requested():
            print "Move preempted"
            # Send preempt request to Controls
            self.vel_client.cancel_goal()
            self._as.set_preempted()
            return

    def handle_imu_heading(self):
        """Update the current heading of the robot."""
        pass

    def handle_pinger_heading(self):
        """Update the estimated pinger heading, and send control commands."""
        # @TODO This will be the callback for the Hydrophones heading message,
        #       in which we will compare it with our stored imu heading and
        #       send the appropriate controls commands to orrient ourselves
        #       toward the pinger
        pass
