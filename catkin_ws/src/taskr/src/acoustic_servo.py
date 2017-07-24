#!/usr/bin/env python
import rospy
from math import fabs
from std_msgs.msg import Float64

from controls.servo_controller import DepthMaintainer
from controls.acoustic_servo import AcousticServo as AcousticController


class AcousticServo(object):
    """Acoustic servo action.

    This action tracks the difference between the current IMU heading and
    the Hydrophones determined heading. It then attempts to move towards
    the pinger.
    """

    PREEMPT_CHECK_FREQUENCY = 1  # Hz
    TIMEOUT = 300  # Seconds = 5 min
    SURGE_ERROR = 10.0

    def __init__(self, data):
        self.acoustic_servo_controller = AcousticController()
        self.depth_maintainer = DepthMaintainer()

        self.preempted = False

    def start(self, server, feedback):
        """Servo toward the pinger.
        Args:
            server:         Action server for publishing feedback.
            feedback_msg:   Feedback message to populate.
        """
        rospy.loginfo("Starting AcousticServo Action")

        self.acoustic_servo_controller.start()
        self.depth_maintainer.start()

        rate = rospy.Rate(self.PREEMPT_CHECK_FREQUENCY)

        # Wait until the robot has stabilized towards the robot
        stable_counts = 0
        while stable_counts < 30:
            rospy.loginfo("{} / 30 consecutive stable periods achieved".format(
                stable_counts))

            if self.preempted:
                self.acoustic_servo_controller.stop()
                self.depth_maintainer.stop()
                return

            err = self.acoustic_servo_controller.get_error()
            if err is None:
                pass
            elif abs(err) < 0.1:
                stable_counts += 1
            else:
                stable_counts = 0
            rospy.sleep(0.1)

        rospy.loginfo("Stabilized towards pinger")
        rospy.loginfo("Approaching pinger")

        # Keep approaching the pinger until we've gone past it
        surge_pub = rospy.Publisher('/controls/superimposer/surge', Float64,
                                    queue_size=1)
        while not self.preempted:
            err = self.acoustic_servo_controller.get_error()
            # If angle is greater than 90, we have reached the pinger.
            if fabs(err) > 1.57:
                rospy.loginfo("Pinger has been reached! Ending")
                break
            else:
                surge_pub.publish(self.SURGE_ERROR)

            rate.sleep()

        surge_pub.publish(0)
        surge_pub.unregister()
        self.acoustic_servo_controller.stop()
        self.depth_maintainer.stop()


    def stop(self):
        self.preempted = True

        self.surge_pub.publish(0)
        self.acoustic_servo_controller.stop()
        self.depth_maintainer.stop()
