#!/usr/bin/env python
import rospy
from math import fabs
from std_msgs.msg import Float64

from controls.servo_controller import DepthMaintainer
from controls.acoustic_servo import AcousticServoController
from controls.utils import normalize_angle


class AcousticServo(object):
    """Acoustic servo action.

    This action tracks the difference between the current IMU heading and
    the Hydrophones determined heading. It then attempts to move towards
    the pinger.
    """

    PREEMPT_CHECK_FREQUENCY = 1  # Hz
    TIMEOUT = 240  # Seconds = 4 min
    SURGE_ERROR = 32.0
    SLOW_SURGE_ERROR = 18.0
    YAW_THRESH = 0.15

    def __init__(self, data):
        self.depth = data["depth"] if "depth" in data else None
        self.acoustic_servo_controller = AcousticServoController()

        if self.depth is None:
            self.depth_maintainer = DepthMaintainer()
        else:
            self.depth_maintainer = DepthMaintainer(self.depth)

        self.surge_pub = rospy.Publisher('/controls/superimposer/surge', Float64,
                                         queue_size=1)

        self.preempted = False

    def start(self, server, feedback):
        """Servo toward the pinger.
        Args:
            server:         Action server for publishing feedback.
            feedback_msg:   Feedback message to populate.
        """
        rospy.loginfo("Starting AcousticServo Action")

        if not self.acoustic_servo_controller.is_active():
            self.acoustic_servo_controller.start()
        if not self.depth_maintainer.is_active():
            self.depth_maintainer.start()

        self.wait_for_depth()

        rate = rospy.Rate(self.PREEMPT_CHECK_FREQUENCY)

        start_time = rospy.Time.now()

        while not self.acoustic_servo_controller.heard_pinger() and not self.preempted:
            if (rospy.Time.now() - start_time).to_sec() > self.TIMEOUT:
                rospy.loginfo("Acoustic servo timed out!")
                break

            rospy.loginfo("Waiting for pinger...")
            self.surge_pub.publish(self.SLOW_SURGE_ERROR)
            rate.sleep()

        rospy.loginfo("Going to the first yaw")

        # Keep approaching the pinger until we've gone past it
        last_heading = None

        self.wait_for_yaw(start_time)

        rospy.loginfo("Approaching pinger")

        while not self.preempted:
            if (rospy.Time.now() - start_time).to_sec() > self.TIMEOUT:
                rospy.loginfo("Acoustic servo timed out!")
                break

            # Message is too old
            if (rospy.Time.now() - self.acoustic_servo_controller.last_heading_time).to_sec() > 8:
                rospy.loginfo("Pinger is lost :(")
                self.surge_pub.publish(self.SLOW_SURGE_ERROR)
                last_heading = None
                continue

            heading = self.acoustic_servo_controller.heading
            if last_heading is None:
                last_heading = heading
                continue
            heading_diff = normalize_angle(heading - last_heading)

            # If angle is greater than 90, we have reached the pinger.
            rospy.loginfo("Err: {}".format(heading_diff))
            if fabs(heading_diff) > 1.3:
                rospy.loginfo("Pinger has been reached! Ending")
                break
            else:
                self.surge_pub.publish(self.SURGE_ERROR)

            rate.sleep()

    def stop(self):
        self.preempted = True

        self.surge_pub.publish(0)

        if self.acoustic_servo_controller.is_active():
            self.acoustic_servo_controller.stop()
        if self.depth_maintainer.is_active():
            self.depth_maintainer.stop()

    def wait_for_depth(self):
        stable_counts = 0
        while stable_counts < 15:
            if self.preempted:
                return

            err = self.depth_maintainer.error
            if err is None:
                pass
            elif abs(err) < 0.3:
                stable_counts += 1
                rospy.loginfo("{} / 15 stable depth periods achieved".format(stable_counts))
            else:
                stable_counts = 0
            rospy.sleep(0.1)

    def wait_for_yaw(self, start_time):
        stable_counts = 0
        while stable_counts < 15:
            if self.preempted:
                return

            if (rospy.Time.now() - start_time).to_sec() > self.TIMEOUT:
                rospy.loginfo("Acoustic servo timed out on wait for yaw!")
                break

            err = self.acoustic_servo_controller.get_error()
            if abs(err) < self.YAW_THRESH:
                stable_counts += 1
                rospy.loginfo("{} / {} stable yaw periods".format(stable_counts, 15))
            else:
                stable_counts = 0
            rospy.sleep(0.1)
