#!/usr/bin/env python

import rospy
import tf

from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

from controls.utils import normalize_angle, SyncServoController, PID, rot_gains


class YawMaintainer(SyncServoController):
    def __init__(self, setpoint=None):
        pid = PID(*rot_gains['yaw'])

        pub = rospy.Publisher('controls/superimposer/yaw',
                              Float64,
                              queue_size=1)

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
                trans, rot = self._listener.lookupTransform('initial_horizon',
                                                            'robot',
                                                            rospy.Time())
                (roll, pitch, yaw) = euler_from_quaternion(rot)
                return yaw
            except (tf.LookupException,
                    tf.ConnectivityException,
                    tf.ExtrapolationException) as e:
                rospy.logdebug(e.message)
                continue

        raise rospy.ROSInterruptException()

    def setpoint_cb(self, msg):
        if (msg is None):
            return

        # TODO Check boundary conditions for acceptable headings;
        # Perform a check
        if (False):
            rospy.logdebug('Received invalid heading')
            return

        if (self.active is False):
            self.start()

        self.set_setpoint(msg.data)


if __name__ == "__main__":
    rospy.init_node('yaw_maintainer')
    maintainer = YawMaintainer()

    sub = rospy.Subscriber('controls/heading', Float64, maintainer.setpoint_cb)

    rospy.loginfo('Yaw maintainer node started')
    rospy.spin()
