#!/usr/bin/env python


import math

import rospy
import tf
from tf import TransformListener
from tf.transformations import euler_from_quaternion


class Process(object):
    '''
    Processes exist to measure the error between the robot's state and a given
    set point.

    Each Process class should provide a get_error method that returns the
    current error (or the best guess based on the most recent measurement)
    '''
    def __init__(self, setpoint):
        self.setpoint = setpoint
        self._timer = None

    def get_error(self):
        '''
        Every process should implement this method. Throws an error if it's not
        '''
        raise NotImplementedError(
            "Process %s must implement the get_error method" %
            self.__class__.__name__)

    def start(self, period, cb):
        '''
        Begins periodic calls to the get_error function and passes the results,
        along with a timer_event, to the provided callback function.

        Use the stop method to end the timer.
        '''
        def callback(timer_event):
            error = self.get_error()
            cb(error, timer_event)
        self._timer = rospy.Timer(period, callback)

    def stop(self):
        '''
        Stops a timer if it has been started. A ValueError will be raised if
        the process has not been started
        '''
        if self._timer is None:
            raise ValueError(
                "Process %s must be started before it can be stopped" %
                self.__class__.__name__)
        self._timer.shutdown()
        self._timer = None


class DepthProcess(Process):
    '''
    Uses transform lookups to calculate the error between the current depth and
    a depth setpoint. N.B. positive depth is downwards / away from the surface
    '''
    def __init__(self, setpoint=None):
        self.listener = TransformListener()
        self.setpoint = setpoint
        if setpoint is None:
            self.setpoint = self._get_current_depth()

        super(DepthProcess, self).__init__(self.setpoint)

    def _get_current_depth(self):
        '''
        Handles the transform lookup to get the current depth
        '''
        while not rospy.is_shutdown():
            try:
                trans, rot = self.listener.lookupTransform(
                    '/floating_horizon', '/robot', rospy.Time())
                return trans[2]
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException):
                continue
        raise rospy.ROSInterruptException()

    def get_error(self):
        estimated_depth = self._get_current_depth()
        return [self.setpoint - estimated_depth]


class YawProcess(Process):
    '''
    Uses transform lookups to calculate the error between the current yaw and a
    yaw setpoint.

    N.B. we use a right hand rule with thumb pointed downward. 0 rad is the
    initialization direction. All yaw is between -pi and pi
    '''
    def __init__(self, setpoint=None):
        self.listener = TransformListener()
        self.setpoint = setpoint
        if setpoint is None:
            self.setpoint = self._get_current_yaw()

        super(YawProcess, self).__init__(self.setpoint)

    def _get_current_yaw(self):
        while not rospy.is_shutdown():
            try:
                trans, rot = self.listener.lookupTransform(
                    'initial_horizon', 'robot', rospy.Time())
                (roll, pitch, yaw) = euler_from_quaternion(rot)
                return yaw
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException):
                continue
        raise rospy.ROSInterruptException()

    @staticmethod
    def _normalize_angle(angle):
        angle = angle % (2 * math.pi)
        if angle > math.pi:
            angle -= 2 * math.pi
        return angle

    def get_error(self):
        estimated_yaw = self._get_current_yaw()
        yaw_error = self.setpoint - estimated_yaw
        return [YawProcess._normalize_angle(yaw_error)]
