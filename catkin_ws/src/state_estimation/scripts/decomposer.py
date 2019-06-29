#!/usr/bin/env python

import rospy
import tf

from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion


class Decomposer:
    def __init__(self):
        rollPub = rospy.Publisher(
                '/state_estimation/roll', Float64, queue_size=1)
        pitchPub = rospy.Publisher(
                '/state_estimation/pitch', Float64, queue_size=1)
        yawPub = rospy.Publisher(
                '/state_estimation/yaw', Float64, queue_size=1)

        self._listener = tf.TransformListener()

    def update_eulers(self):
        try:
            trans, rot = self._listener.lookupTransform(
                    'initial_horizon', 'robot', rospy.Time(0))

            (roll, pitch, yaw) = euler_from_quaternion(rot)
            
            return

        except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException) as e:
            rospy.logdebug(e.message)

    def publish(self):
        self.rollPub.publish(roll)
        self.pitchPub.publish(pitch)
        self.yawPub.publish(yaw)


if __name__ == "__main__":
    rospy.init_node('decomposer')
    decomposer = Decomposer()
    
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        decomposer.update_eulers()
        decomposer.publish()
        rospy.spin()
