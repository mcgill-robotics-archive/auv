import rospy

import tf
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import PolygonStamped, Polygon
from darknet_ros_msgs.msg import BoundingBoxes
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point

from PID import PID, trans_gains, rot_gains
from utils import normalize_angle

CAM_OFFSET = 1.18

PROB_THRESH = 0.5

IMAGE_WIDTH = 1296
IMAGE_HEIGHT = 964
IMAGE_CENTER_X = IMAGE_WIDTH / 2
IMAGE_CENTER_Y = IMAGE_HEIGHT / 2


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


""" -------------------------- DEPTH MAINTAINER -------------------------- """


class DepthMaintainer(AsyncServoController):
    def __init__(self, setpoint=None):
        pid = PID(*trans_gains['heave'])
        pub = rospy.Publisher('controls/superimposer/heave', Float64,
                              queue_size=1)
        self.error = None

        if setpoint is None:
            setpoint = rospy.wait_for_message('state_estimation/depth',
                                              Float64, timeout=5.0).data

        super(DepthMaintainer, self).__init__(
            pid, pub, 'state_estimation/depth', Float64, setpoint)

    def get_error(self, msg):
        self.error = self.setpoint - msg.data
        return self.error


""" --------------------------- YAW MAINTAINER --------------------------- """


class YawMaintainer(SyncServoController):
    def __init__(self, setpoint=None):
        pid = PID(*rot_gains['yaw'])
        pub = rospy.Publisher(
            'controls/superimposer/yaw', Float64, queue_size=1)

        self._listener = tf.TransformListener()

        if setpoint is None:
            setpoint = self.get_current_yaw()
        super(YawMaintainer, self).__init__(pid, pub, setpoint)

    def get_error(self):
        estimated_yaw = self.get_current_yaw()
        yaw_error = self.setpoint - estimated_yaw
        return normalize_angle(yaw_error)

    def get_current_yaw(self):
        while not rospy.is_shutdown():
            try:
                trans, rot = self._listener.lookupTransform(
                    'initial_horizon', 'robot', rospy.Time())
                (roll, pitch, yaw) = euler_from_quaternion(rot)
                return yaw
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException) as e:
                continue
        raise rospy.ROSInterruptException()


""" -------------------------------- BUOYS -------------------------------- """


class BuoyVisualServoMaintainer(object):
    def __init__(self, setpoint):
        self.servo_sway = BuoyServoSway()
        self.servo_heave = BuoyServoHeave()

    def start(self):
        self.servo_sway.start()
        self.servo_heave.start()

    def stop(self):
        self.servo_sway.stop()
        self.servo_heave.stop()

    def get_error(self):
        return (self.servo_sway.error, self.servo_heave.error)


class BuoyServoSway(AsyncServoController):
    def __init__(self, setpoint=IMAGE_CENTER_X):
        self.pid = PID(*trans_gains['sway'])
        self.pub = rospy.Publisher(
            'controls/superimposer/sway', Float64, queue_size=1)

        self.error = None
        self.aimed = IMAGE_CENTER_X

        super(BuoyServoSway, self).__init__(
            self.pid, self.pub, '/darknet_ros_topic',
            BoundingBoxes, setpoint)

    def get_error(self, boxes):
        # Assumes that buoys will occupy the first index
        box = boxes[0]

        if box.probability > PROB_THRESH:
            gravity_x = (box.xmin + box.xmax) / 2
            self.error = gravity_x - self.aimed
        else:
            self.error = None

        return self.error


class BuoyServoHeave(AsyncServoController):
    def __init__(self, setpoint=IMAGE_CENTER_Y):
        self.pid = PID(*trans_gains['heave'])
        self.pub = rospy.Publisher(
            'controls/superimposer/heave', Float64, queue_size=1)

        self.error = None
        self.aimed = IMAGE_CENTER_Y

        super(BuoyServoHeave, self).__init__(
            self.pid, self.pub, '/darknet_ros_topic',
            BoundingBoxes, setpoint)

    def get_error(self, boxes):
        # Assumes that buoys will occupy the first index
        box = boxes[0]

        if box.probability > PROB_THRESH:
            gravity_y = (box.ymin + box.ymax) / 2
            self.error = gravity_y - self.aimed
        else:
            self.error = None

        return self.error


""" --------------------------- TORPEDO TARGET --------------------------- """


class TorpedoVisualServoMaintainer(object):
    def __init__(self, setpoint):
        self.servo_sway = TorpedoServoSway()
        self.servo_heave = TorpedoServoHeave()

    def start(self):
        self.servo_sway.start()
        self.servo_heave.start()

    def stop(self):
        self.servo_sway.stop()
        self.servo_heave.stop()

    def get_error(self):
        return (self.servo_sway.error, self.servo_heave.error)


class TorpedoServoSway(AsyncServoController):
    def __init__(self, setpoint=IMAGE_CENTER_X):
        self.pid = PID(*trans_gains['sway'])
        self.pub = rospy.Publisher(
            'controls/superimposer/sway', Float64, queue_size=1)

        self.error = None
        self.aimed = setpoint

        super(BuoyServoSway, self).__init__(
            self.pid, self.pub, '/state_estimation/torpedo_targeter',
            PolygonStamped, setpoint)

    def get_error(self, msg):
        poly = msg.polygon

        gravity_x = (poly[0].x + poly[1].x + poly[2].x + poly[3].x) / 4
        self.error = gravity_x - self.aimed

        return self.error


class TorpedoServoHeave(AsyncServoController):
    def __init__(self, setpoint=IMAGE_CENTER_Y):
        self.pid = PID(*trans_gains['heave'])
        self.pub = rospy.Publisher(
            'controls/superimposer/heave', Float64, queue_size=1)

        self.error = None
        self.aimed_y = setpoint

        super(BuoyServoSway, self).__init__(
            self.pid, self.pub, '/state_estimation/torpedo_targeter',
            PolygonStamped, setpoint)

    def get_error(self, msg):
        poly = msg.polygon

        gravity_y = (poly[0].y + poly[1].y + poly[2].y + poly[3].y) / 4
        self.error = gravity_y - self.aimed

        return self.error


""" -------------------------------- BINS -------------------------------- """


class BinsVisualServoMaintainer(object):
    def __init__(self, setpoint=None):
        self.servo_sway = BinsServoSway()
        self.servo_surge = BinsServoSurge()
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
        if (self.servo_sway.error is None) or (self.servo_surge.error is None):
            return (None, None)

        x_0 = self.servo_sway.error
        y_0 = self.servo_surge.error

        x_n = (np.cos(CAM_OFFSET) * x_0) - (np.sin(CAM_OFFSET) * y_0)
        y_n = (np.sin(CAM_OFFSET) * x_0) + (np.cos(CAM_OFFSET) * y_0)

        return (x_n, y_n)

    def is_active(self):
        return self.active


class BinsServoSway(AsyncServoController):
    def __init__(self, setpoint=IMAGE_CENTER_X):
        self.pid = PID(*trans_gains['sway'])
        self.pub = rospy.Publisher(
            'controls/superimposer/sway', Float64, queue_size=1)

        self.error = None
        self.aimed_x = setpoint

        super(BinsServoSway, self).__init__(
            self.pid, self.pub, '/state_estimation/bins',
            PolygonStamped, setpoint)

    def get_error(self, msg):
        poly = msg.polygon.points

        if len(poly) == 0:
            return None

        gravity_x = (poly[0].x + poly[1].x + poly[2].x + poly[3].x) / 4
        self.error = gravity_x - self.aimed_x

        return self.error


class BinsServoSurge(AsyncServoController):
    def __init__(self, setpoint=IMAGE_CENTER_Y):
        self.pid = PID(*trans_gains['surge'])
        self.pub = rospy.Publisher(
            'controls/superimposer/surge', Float64, queue_size=1)

        self.error = None
        self.aimed_y = setpoint

        super(BinsServoSurge, self).__init__(
            self.pid, self.pub, '/state_estimation/bins',
            PolygonStamped, setpoint)

    def get_error(self, msg):
        poly = msg.polygon.points

        if len(poly) == 0:
            return None

        for ele in poly:
            ele.y = 964 - ele.y

        gravity_y = (poly[0].y + poly[1].y + poly[2].y + poly[3].y) / 4
        self.error = self.aimed_y - gravity_y

        return self.error


def transform_polygon(poly, x, y, yaw):
    """Should be a utility function. Rotate a polygon as follows:

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
