#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

from PID import PID, trans_gains, rot_gains
from utils import get_depth_error, get_yaw_error, get_current_yaw, get_current_depth


class ServoController(object):
    """This is a controller used to servo (drive an error to zero) in one axis.
    Inherit from it for simple controllers, or create a member variable with
    an instance of it to command any quantity to zero."""

    def __init__(self, pid, pub, error_fn, setpoint,
                 period=0.1, stable_periods=10, stable_threshold=0.1):
        """Constructor for the ServoController class.

        Args:
            pid: PID object to use.
            pub: Publisher to publish to. Must be compatible with PID output
                 (ie, must be one of the superimposer topics.
            error_fn: A function which returns the error between the current
                      state and the desired setpoint.
            setpoint: The setpoint to go to.
        """
        self.pid = pid
        self.pub = pub
        self.setpoint = setpoint
        self.error_fn = error_fn

        self.stable_periods = stable_periods
        self.stable_threshold = stable_threshold
        self.period = period

        self.update_timer = None

    def _update(self, event):
        res = self.pid.update(self.error_fn(self.setpoint), self.period)
        self.pub.publish(res)

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint

    def start(self):
        if self.update_timer is None:
            self.update_timer = rospy.Timer(rospy.Duration(self.period), self._update)

        cnt = 0
        while cnt < self.stable_periods:
            if abs(self.error_fn(self.setpoint)) < self.stable_threshold:
                cnt += 1
            else:
                cnt = 0
            rospy.sleep(self.period)

    def stop(self, reset=True):
        if reset:
            self.pid.reset()
        self.update_timer.shutdown()
        self.update_timer = None


class DepthMaintainer(ServoController):
    def __init__(self, setpoint=None):
        pid = PID(*trans_gains['heave'])
        pub = rospy.Publisher('controls/superimposer/heave', Float64, queue_size=1)
        if setpoint is None:
            setpoint = get_current_depth()
        super(DepthMaintainer, self).__init__(pid, pub, get_depth_error, setpoint)


class YawMaintainer(ServoController):
    def __init__(self, setpoint=None):
        pid = PID(*rot_gains['yaw'])
        pub = rospy.Publisher('controls/superimposer/yaw', Float64, queue_size=1)
        if setpoint is None:
            setpoint = get_current_yaw()
        super(YawMaintainer, self).__init__(pid, pub, get_yaw_error, setpoint)
