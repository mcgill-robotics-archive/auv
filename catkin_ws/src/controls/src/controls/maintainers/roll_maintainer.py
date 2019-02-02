#!/usr/bin/env python

import rospy
import tf

from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

from controls.utils import normalize_angle, SyncServoController, PID, rot_gains


class RollMaintainer(SyncServoController):
    def __init__(self, setpoint=None):
        pid = PID(*rot_gains['roll'])

        pub = rospy.Publisher('controls/superimposer/roll',
                              Float64,
                              queue_size=1)

        self._listener = tf.TransformListener()

        if setpoint is None:
            setpoint = self.get_current_roll()

        super(RollMaintainer, self).__init__(pid, pub, setpoint)

    def get_error(self):
        estimated_roll = self.get_current_roll()
        roll_error = self.setpoint - estimated_roll
        return normalize_angle(roll_error)

    def get_current_roll(self):
        while not rospy.is_shutdown():
            try:
                trans, rot = self._listener.lookupTransform('initial_horizon',
                                                            'robot',
                                                            rospy.Time())
                (roll, pitch, yaw) = euler_from_quaternion(rot)
                return roll
            except (tf.LookupException,
                    tf.ConnectivityException,
                    tf.ExtrapolationException) as e:
                rospy.logdebug(e.message)
                continue

        raise rospy.ROSInterruptException()

    def setpoint_cb(self, msg):
        if (msg is None):
            return

        # TODO Check boundary conditions for acceptable rolls;
        # Perform a check
        if (False):
            rospy.logdebug('Received invalid roll')
            return

        self.set_setpoint(msg.data)


if __name__ == "__main__":
    rospy.init_node('roll_maintainer')
    maintainer = RollMaintainer()

    sub = rospy.Subscriber('controls/roll', Float64, maintainer.setpoint_cb)

    rospy.loginfo('Roll maintainer node started')
    rospy.spin()
