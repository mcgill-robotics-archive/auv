#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

from process import DepthProcess, YawProcess
from PID import PID, trans_gains, rot_gains


class Controller(object):
    def __init__(self, pids, pubs, process, setpoint=None):
        self.pids = pids
        self.pubs = pubs
        self.process = process
        self.setpoint = setpoint

    @property
    def setpoint(self):
        return self.process.setpoint

    @setpoint.setter
    def setpoint(self, value):
        self.process.setpoint = value

    @property
    def error(self):
        return self.process.get_error()

    def _update(self, error, timer_event):
        if timer_event.last_duration is None:
            timer_event.last_duration = 0.1
        i = 0
        for pid, pub in zip(self.pids, self.pubs):
            res = pid.update(error[i], timer_event.last_duration)
            pub.publish(res)
            i += 1

    def start(self, period):
        self.process.start(period, self._update)

    def stop(self, reset=True):
        if reset:
            for pid in self.pids:
                pid.reset()
        self.process.stop()


class DepthMaintainer(Controller):
    def __init__(self, setpoint=None):
        heave_pid = PID(*trans_gains['heave'])
        heave_pub = rospy.Publisher('/controls/superimposer/heave', Float64,
                                    queue_size=10)
        process = DepthProcess(setpoint)
        pids = [heave_pid]
        pubs = [heave_pub]
        super(DepthMaintainer, self).__init__(pids, pubs, process, setpoint)


class YawMaintainer(Controller):
    def __init__(self, setpoint=None):
        yaw_pid = PID(*rot_gains['yaw'])
        yaw_pub = rospy.Publisher('/controls/superimposer/yaw', Float64,
                                  queue_size=10)
        process = YawProcess(setpoint)
        pids = [yaw_pid]
        pubs = [yaw_pub]
        super(YawMaintainer, self).__init__(pids, pubs, process, setpoint)
