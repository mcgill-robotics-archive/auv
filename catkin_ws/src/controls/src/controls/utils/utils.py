import rospy
import math
import numpy as np

from geometry_msgs.msg import Polygon, Point


class SyncServoController(object):
    """
    SyncServoController is a convenience class for periodically updating a
    PID with an error function and publishing the result to some topic.

    Use this when you have a synchronous/periodic measurement to make.

    Args:
        pid (PID): PID to update
        pub (Publisher): publisher to which the PID response will be posted
        setpoint (float): desired target value

    Attributes:
        pid (PID): PID to update
        pub (Publisher): publisher to which the PID response will be posted
        setpoint (float): desired target value
    """
    def __init__(self, pid, pub, setpoint):
        self.pid = pid
        self.pub = pub
        self.setpoint = setpoint

        self._update_timer = None

        self.active = False

    def _update(self, timer_event):
        if timer_event.last_expected is None:
            duration = 0.1
        else:
            now = rospy.Time.now()
            duration = (now - timer_event.last_expected).to_sec()

        error = self.get_error()
        res = self.pid.update(error, duration)
        self.pub.publish(res)

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
        self.pid.reset()

    def get_error(self):
        """
        get_error is the method implemented to report the error from the
        current position to the setpoint, i.e. setpoint - state

        All subclasses must implement this method
        """
        raise NotImplementedError(
            "%s must implement the get_error method" %
            self.__class__.__name__)

    def start(self, period=0.1):
        """
        start starts the servoing process. The update period can be specified
        with the period argument

        start will raise an exception if there is an attempt to start a
        SyncServoController before it has been stopped

        Args:
            period (float): the period at which to update and publish the
            controller response. Defaults to None
        """
        if self._update_timer is not None:
            raise ValueError(
                "Tried to start %s but it was already started" %
                self.__class__.__name__)

        self.active = True
        self._update_timer = rospy.Timer(
            rospy.Duration(period), self._update)

    def stop(self):
        """
        stop stops the servoing process

        stop will raise an exception if there is an attempt to stop a
        SyncServoController before it has been started
        """
        if self._update_timer is None:
            raise ValueError(
                "Tried to stop %s but it hasn't been started" %
                self.__class__.__name__)

        self.active = False
        self.pub.publish(0.0)
        self._update_timer.shutdown()
        self._update_timer = None

    def is_active(self):
        """Returns True if active, False if stopped."""
        return self.active


class AsyncServoController(object):
    """
    ASyncServoController is a convenience class for updating a PID with an
    error function and publishing the result to some topic in response to a
    subscription from a topic

    Use this when you have a topic that you wish to respond to with a
    controller

    Args:
        pid (PID): PID to update
        pub (Publisher): publisher to which the PID response will be posted
        sub_topic (str): the name of the topic to subscribe to
        sub_data_class (any): the data type class to use for messages, e.g.
            std_msgs.Float64
        setpoint (float): desired target value

    Attributes:
        pid (PID): PID to update
        pub (Publisher): publisher to which the PID response will be posted
        sub_topic (str): the name of the topic to subscribe to
        sub_data_class (any): the data type class to use for messages, e.g.
            std_msgs.Float64
        setpoint (float): desired target value
    """
    def __init__(self, pid, pub, sub_topic, sub_data_class, setpoint):
        self.pid = pid
        self.pub = pub
        self.setpoint = setpoint

        self.sub_topic = sub_topic
        self.sub_data_class = sub_data_class

        self._sub = None
        self._last_event = None

        self.active = False

    def _update(self, msg):
        now = rospy.Time.now().to_sec()
        last_duration = 0.1
        if self._last_event is not None:
            last_duration = now - self._last_event
        self._last_event = now

        error = self.get_error(msg)

        if error is None:
            self.pid.reset()
            self.pub.publish(0)
            return

        res = self.pid.update(error, last_duration)
        self.pub.publish(res)

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
        self.pid.reset()

    def get_error(self, msg):
        """
        get_error is the method implemented to report the error from the
        current position to the setpoint, i.e. setpoint - state. The sub_topic
        provides the data inputted to msg.

        All subclasses must implement this method
        """
        raise NotImplementedError(
            "%s must implement the get_error method" %
            self.__class__.__name__)

    def start(self):
        """
        start starts the servoing process

        start will raise an exception if there is an attempt to start a
        AsyncServoController before it has been stopped
        """
        if self._sub is not None:
            raise ValueError("Tried to start %s but it was already started" %
                             self.__class__.__name__)

        self.active = True
        self._sub = rospy.Subscriber(self.sub_topic, self.sub_data_class,
                                     self._update, queue_size=1)

    def stop(self):
        """
        stop stops the servoing process

        stop will raise an exception if there is an attempt to stop a
        AsyncServoController before it has been started
        """
        if self._sub is None:
            raise ValueError("Tried to stop %s but it wasn't started" %
                             self.__class__.__name__)

        self.active = False
        self.pub.publish(0.0)
        self._sub.unregister()

    def is_active(self):
        """Returns True if active, False if stopped."""
        return self.active


def normalize_angle(angle):
    """
    normalizes an angle to the interval (-pi, pi]

    Args:
        angle (float): an angle in radians

    Returns:
        float: the angle normalized onto (-pi, pi]
    """
    angle = angle % (2 * math.pi)
    if angle > math.pi:
        angle -= 2 * math.pi
    elif angle <= -math.pi:
        angle += 2 * math.pi
    return angle


def transform_polygon(poly, x, y, yaw):
    """Rotate a polygon as follows:

        P_new = P * R

    where P and P_new are of the form [[x1, x2 ... xn], [y1, y2 ... yn]]
    and R is [[cos(yaw), -sin(yaw)], [sin(yaw), cos(yaw)]]."""
    transformed = Polygon()

    for ele in poly.points:
        ele.y = 964 - ele.y
        ele.x += x
        ele.y += y

    for ele in poly.points:
        pt = Point()
        pt.x = ele.x * np.cos(yaw) - ele.y * np.sin(yaw)
        pt.y = ele.x * np.sin(yaw) + ele.y * np.cos(yaw)
        transformed.points.append(pt)

    # return poly
    return transformed
