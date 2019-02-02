#!/usr/bin/env python
import rospy
from math import fabs
from std_msgs.msg import Float64
from controls.maintainers import DepthMaintainer
from controls.controllers import SonarServoController
from controls.utils import normalize_angle


class SonarServo(object):
    """Sonar servo action.

    This action tracks the difference between the current IMU heading and
    the TaskPoint heading. It then attempts to move towards
    the TaskPoint.
    """

    PREEMPT_CHECK_FREQUENCY = 1  # Hz
    TIMEOUT = 300  # Seconds = 5 min
    SURGE_ERROR = 35.0

    def __init__(self, data):
        self.depth = data["depth"] if "depth" in data else None
        self.sonar_servo_controller = SonarServoController()

        if self.depth is None:
            self.depth_maintainer = DepthMaintainer()
        else:
            self.depth_maintainer = DepthMaintainer(self.depth)

        self.surge_pub = rospy.Publisher('/controls/superimposer/surge', Float64,
                                         queue_size=1)

        self.preempted = False

    def start(self, server, feedback):
        """Servo toward the TaskPoint.
        Args:
            server:         Action server for publishing feedback.
            feedback_msg:   Feedback message to populate.
        """
        rospy.loginfo("Starting SonarServo Action")

        if not self.sonar_servo_controller.is_active():
            self.sonar_servo_controller.start()
        if not self.depth_maintainer.is_active():
            self.depth_maintainer.start()

        self.wait_for_depth()

        rate = rospy.Rate(self.PREEMPT_CHECK_FREQUENCY)

        while not self.sonar_servo_controller.saw_TaskPoint() and not self.preempted:
            rospy.loginfo("Waiting for TaskPoint...")
            rate.sleep()

        rospy.loginfo("Approaching TaskPoint")

        # Keep approaching the TaskPoint until we've gone past it
        last_heading = None
        while not self.preempted:
            heading = self.sonar_servo_controller.heading
            if last_heading is None:
                last_heading = heading
                continue
            heading_diff = normalize_angle(heading - last_heading)

            # If angle is greater than 90, we have reached the pinger.
            rospy.loginfo("Err: {}".format(heading_diff))
            if fabs(heading_diff) > 1.3:
                rospy.loginfo("TaskPoint has been reached! Ending")
                break
            else:
                self.surge_pub.publish(self.SURGE_ERROR)

            rate.sleep()

    def stop(self):
        self.preempted = True

        self.surge_pub.publish(0)

        if self.sonar_servo_controller.is_active():
            self.sonar_servo_controller.stop()
        if self.depth_maintainer.is_active():
            self.depth_maintainer.stop()

    def wait_for_depth(self):
        stable_counts = 0
        while stable_counts < 15:
            rospy.loginfo("{} / 30 stable periods achieved".format(
                stable_counts))

            if self.preempted:
                return

            err = self.depth_maintainer.error
            if err is None:
                pass
            elif abs(err) < 0.3:
                stable_counts += 1
            else:
                stable_counts = 0
            rospy.sleep(0.1)
