#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Float64

from cv.msg import CvTarget
from controls.utils import AsyncServoController, PID, trans_gains


class FrontVisualServoController(object):
    def __init__(self, setpoint=None):
        # Get Params
        self.params.cam_offset = rospy.get_param('/cameras/offset', 0)
        self.params.prob_thresh = rospy.get_param('/cameras/prob_thresh', 0.5)
        self.params.img_width = rospy.get_param('/cameras/img_width', 1296)
        self.params.img_height = rospy.get_param('/cameras/img_height', 964)

        self.img_center_x = self.img_width / 2
        self.img_center_y = self.img_height / 2

        self.params.is_front = True

        # Initialize Visual Servo Axis Controllers
        self.servo_sway = VisualServoSway(self.params)
        self.servo_heave = VisualServoHeave(self.params)

        self.active = False

    def start(self):
        self.servo_sway.start()
        self.servo_heave.start()
        self.active = True

    def stop(self):
        self.servo_sway.stop()
        self.servo_heave.stop()
        self.active = False

    def get_error(self):
        # ALL THIS SHOULDN'T BE NECESSARY WITH PROPER MECH INSTALLATION
        # EVENTUALLY CAN BE REDUCED TO JUST RETURNING THE ERROR
        if (self.servo_sway.error is None) or (self.servo_surge.error is None):
            return (None, None)

        x_0 = self.servo_sway.error
        y_0 = self.servo_heave.error

        x_n = ((np.cos(self.params.cam_offset) * x_0) -
               (np.sin(self.params.cam_offset) * y_0))

        y_n = ((np.sin(self.params.cam_offset) * x_0) +
               (np.cos(self.params.cam_offset) * y_0))

        return (x_n, y_n)

    def is_active(self):
        return self.active


class DownVisualServoController(object):
    def __init__(self, setpoint=None):
        # Get Params
        self.params.cam_offset = rospy.get_param('/cameras/offset', 1.18)
        self.params.prob_thresh = rospy.get_param('/cameras/prob_thresh', 0.5)
        self.params.img_width = rospy.get_param('/cameras/img_width', 1296)
        self.params.img_height = rospy.get_param('/cameras/img_height', 964)

        self.img_center_x = self.img_width / 2
        self.img_center_y = self.img_height / 2

        self.params.is_front = True

        # Initialize Visual Servo Axis Controllers
        self.servo_sway = VisualServoSway(self.params)
        self.servo_surge = VisualServoSurge(self.params)

        self.active = False

    def start(self):
        self.servo_sway.start()
        self.servo_surge.start()
        self.active = True

    def stop(self):
        self.servo_sway.stop()
        self.servo_surge.stop()
        self.active = False

    def get_error(self):
        # ALL THIS SHOULDN'T BE NECESSARY WITH PROPER MECH INSTALLATION
        # EVENTUALLY CAN BE REDUCED TO JUST RETURNING THE ERROR
        if (self.servo_sway.error is None) or (self.servo_surge.error is None):
            return (None, None)

        x_0 = self.servo_sway.error
        y_0 = self.servo_surge.error

        x_n = ((np.cos(self.params.cam_offset) * x_0) -
               (np.sin(self.params.cam_offset) * y_0))

        y_n = ((np.sin(self.params.cam_offset) * x_0) +
               (np.cos(self.params.cam_offset) * y_0))

        return (x_n, y_n)

    def is_active(self):
        return self.active


class VisualServoSurge(AsyncServoController):
    def __init__(self, params, setpoint=None):
        self.params = params

        if setpoint is None:
            setpoint = self.params.img_center_y

        self.pid = PID(*trans_gains['surge'])
        self.pub = rospy.Publisher('controls/superimposer/surge',
                                   Float64,
                                   queue_size=1)

        self.error = None
        self.aimed_y = setpoint

        super(VisualServoSurge, self).__init__(self.pid,
                                               self.pub,
                                               'cv/down_cam_target',
                                               CvTarget,
                                               setpoint)

    def get_error(self, msg):
        if msg.probability > self.params.prob_thresh:
            self.error = msg.gravity.y - self.aimed_y
        else:
            self.error = None

        return self.error


class VisualServoSway(AsyncServoController):
    def __init__(self, params, setpoint=None):
        self.params = params

        if setpoint is None:
            setpoint = self.params.img_center_x

        self.pid = PID(*trans_gains['sway'])
        self.pub = rospy.Publisher('controls/superimposer/sway',
                                   Float64,
                                   queue_size=1)

        self.error = None
        self.aimed_x = setpoint

        if self.params.is_front:
            super(VisualServoSway, self).__init__(self.pid,
                                                  self.pub,
                                                  'cv/front_cam_target',
                                                  CvTarget,
                                                  setpoint)
        else:
            super(VisualServoSway, self).__init__(self.pid,
                                                  self.pub,
                                                  'cv/down_cam_target',
                                                  CvTarget,
                                                  setpoint)

    def get_error(self, msg):
        if msg.probability > self.params.prob_thresh:
            self.error = msg.gravity.x - self.aimed_x
        else:
            self.error = None

        return self.error


class VisualServoHeave(AsyncServoController):
    def __init__(self, params, setpoint=None):
        self.params = params

        if setpoint is None:
            setpoint = self.params.img_center_y

        self.pid = PID(*trans_gains['heave'])
        self.pub = rospy.Publisher('controls/superimposer/heave',
                                   Float64,
                                   queue_size=1)

        self.error = None
        self.aimed_y = setpoint

        super(VisualServoHeave, self).__init__(self.pid,
                                               self.pub,
                                               '/cv/front_cam_target',
                                               CvTarget,
                                               setpoint)

    def get_error(self, msg):
        if msg.probability > self.params.prob_thresh:
            self.error = msg.gravity.y - self.aimed_y
        else:
            self.error = None

        return self.error
