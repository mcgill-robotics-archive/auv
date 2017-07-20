import tf
import rospy
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

from PID import PID, trans_gains, rot_gains
from utils import normalize_angle


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

    def _update(self, timer_event):
        if timer_event.last_expected is None:
            duration = 0.1
        else:
            now = rospy.Time.now()
            duration = (now - timer_event.last_expected).to_sec()

        error = self.get_error()
        res = self.pid.update(error, duration)
        self.pub.publish(res)

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
            raise ValueError("Tried to start %s but it was already started" %
                          self.__class__.__name__)

        self._update_timer = rospy.Timer(rospy.Duration(period),
                                        self._update)

    def stop(self):
        """
        stop stops the servoing process

        stop will raise an exception if there is an attempt to stop a
        SyncServoController before it has been started
        """
        if self._update_timer is None:
            raise ValueError("Tried to stop %s but it hasn't been started" %
                             self.__class__.__name__)

        self.pub.publish(0.0)
        self._update_timer.shutdown()
        self._update_timer = None


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

    def _update(self, msg):
        now = rospy.Time.now().to_sec()
        last_duration = 0.1
        if self._last_event is not None:
            last_duration = now - self._last_event
        self._last_event = now

        error = self.get_error(msg)
        res = self.pid.update(error, last_duration)
        self.pub.publish(res)

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

        self.pub.publish(0.0)
        self._sub.unregister()


class DepthMaintainer(AsyncServoController):
    def __init__(self, setpoint=None):
        pid = PID(*trans_gains['heave'])
        pub = rospy.Publisher('controls/superimposer/heave', Float64,
                              queue_size=1)

        if setpoint is None:
            setpoint = 0.0

        super(DepthMaintainer, self).__init__(
            pid, pub, 'state_estimation/depth', Float64, setpoint)

    def get_error(self, msg):
        return self.setpoint - msg.data


class YawMaintainer(SyncServoController):
    def __init__(self, setpoint=None):
        pid = PID(*rot_gains['yaw'])
        pub = rospy.Publisher('controls/superimposer/yaw', Float64, queue_size=1)

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
