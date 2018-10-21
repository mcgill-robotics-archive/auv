#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

from controls.maintainers import DepthMaintainer, YawMaintainer#, BinsVisualServoMaintainer
from std_srvs.srv import SetBool


class BinServo(object):
    """Bin servo action.

    This action tracks the difference between the current image center and the
    detected bin center
    """

    PREEMPT_CHECK_FREQUENCY = 1  # Hz
    TIMEOUT = 60  # Seconds = 1 min
    SURGE_ERROR = 35.0

    def __init__(self, data):
        self.bin_servo_controller = BinsVisualServoMaintainer()

        self.depth = data["depth"] if "depth" in data else None
        self.depth_maintainer = DepthMaintainer(self.depth)

        self.yaw = data["yaw"] if "yaw" in data else None
        self.yaw_maintainer = YawMaintainer(self.yaw)

        self.surge_pub = rospy.Publisher('/controls/superimposer/surge', Float64,
                                         queue_size=1)

        self.set_bin_state = "bin_detector/set_state"
        rospy.wait_for_service(self.set_bin_state)

        try:
            set_state = rospy.ServiceProxy(self.set_bin_state, SetBool)
            response = set_state(True)
            rospy.loginfo("Bin turned on: {}".format(response))
        except rospy.ServiceException as e:
            print "Service call failed: {}".format(e)

        self.preempted = False

    def start(self, server, feedback):
        """Servo towards the center of the bin
        Args:
            server:         Action server for publishing feedback.
            feedback_msg:   Feedback message to populate.
        """
        rospy.loginfo("Starting BinsServo Action")

        if not self.depth_maintainer.is_active():
            self.depth_maintainer.start()
        if not self.yaw_maintainer.is_active():
            self.yaw_maintainer.start()

        self.wait_for_depth()
        self.wait_for_yaw()

        rospy.loginfo("Servoing over bin")
        if not self.bin_servo_controller.is_active():
            self.bin_servo_controller.start()

        self.wait_for_servo()

    def stop(self):
        self.preempted = True

        if self.bin_servo_controller.is_active():
            self.bin_servo_controller.stop()
        if self.yaw_maintainer.is_active():
            self.yaw_maintainer.stop()
        if self.depth_maintainer.is_active():
            self.depth_maintainer.stop()

        rospy.wait_for_service(self.set_bin_state)

        try:
            set_state = rospy.ServiceProxy(self.set_bin_state, SetBool)
            response = set_state(False)
            rospy.loginfo("Bin turned off: {}".format(response))
        except rospy.ServiceException as e:
            print "Service call failed: {}".format(e)

    def wait_for_depth(self):
        rospy.loginfo("Going to depth")

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

        rospy.loginfo("Finished depth")

    def wait_for_yaw(self):
        rospy.loginfo("Going to yaw")

        stable_counts = 0
        while stable_counts < 15:
            rospy.loginfo("{} / 15 stable periods achieved".format(
                stable_counts))

            if self.preempted:
                return

            err = self.yaw_maintainer.get_error()
            if err is None:
                pass
            elif abs(err) < 0.3:
                stable_counts += 1
            else:
                stable_counts = 0
            rospy.sleep(0.1)

        rospy.loginfo("Finished yaw")

    def wait_for_servo(self):
        rospy.loginfo("Servoing over bin")

        start_time = rospy.Time.now()

        stable_counts = 0
        while stable_counts < 30:
            if (rospy.Time.now() - start_time).to_sec() > self.TIMEOUT:
                rospy.loginfo("Bins servo timed out!")
                break

            rospy.loginfo("{} / 15 stable periods achieved".format(
                stable_counts))

            if self.preempted:
                return

            (x_err, y_err) = self.bin_servo_controller.get_error()
            if x_err is None or y_err is None:
                pass
            elif abs(x_err) < 300 and abs(y_err) < 300:
                stable_counts += 1
            else:
                stable_counts = 0
            rospy.sleep(0.1)

        rospy.loginfo("Finished bin servo")
