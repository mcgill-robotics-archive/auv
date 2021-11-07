#!/usr/bin/env python

import rospy
import tf

from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion


class Decomposer:
    def __init__(self):
        self.rollPub = rospy.Publisher(
                '/state_estimation/roll', Float64, queue_size=1)
        self.pitchPub = rospy.Publisher(
                '/state_estimation/pitch', Float64, queue_size=1)
        self.yawPub = rospy.Publisher(
                '/state_estimation/yaw', Float64, queue_size=1)

        self._listener = tf.TransformListener()

    def update_eulers(self):
        try:
            rospy.loginfo('PIN')
            trans, rot = self._listener.lookupTransform(
                    'initial_horizon', 'robot', rospy.Time(0))

            (roll, pitch, yaw) = euler_from_quaternion(rot)

            self.rollPub.publish(roll)
            self.pitchPub.publish(pitch)
            self.yawPub.publish(yaw)
            return

        except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException) as e:
            rospy.logdebug(e.message)


if __name__ == "__main__":
    rospy.init_node('decomposer')
    decomposer = Decomposer()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        decomposer.update_eulers()
        rate.sleep()
    rospy.spinOnce()
