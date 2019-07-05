#!/usr/bin/env python

import rospy
import tf

from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion


class Decomposer:
    '''
    Decomposer class to monitor tf and publish raw floats to ROS PID
    '''

    def __init__(self):
        '''
        Constructor for Decomposer
        '''

        # Publishers and Subscribers
        self.roll_pub = rospy.Publisher(
                '/state_estimation/roll', Float64, queue_size=1)
        self.pitch_pub = rospy.Publisher(
                '/state_estimation/pitch', Float64, queue_size=1)
        self.yaw_pub = rospy.Publisher(
                '/state_estimation/yaw', Float64, queue_size=1)

        # tf Listener
        self._listener = tf.TransformListener()

        # Relevant tf Frames
        self.base_link_frame = rospy.get_param(
            'state_estimation/base_link_frame', default='robot')
        self.map_frame = rospy.get_param(
            'state_estimation/map_frame', default='initial_horizon')

    def update_eulers(self):
        '''
        Lookup tf transform and convert to euler angles
        '''
        try:
            rospy.loginfo('PIN')
            trans, rot = self._listener.lookupTransform(
                    self.map_frame, self.base_link_frame, rospy.Time(0))

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

    # Decomposer update rate
    rate = rospy.Rate(rospy.get_param('state_estimation/decomposer_rate', 10))

    # Cycle forever at fixed rate
    while not rospy.is_shutdown():
        decomposer.update_eulers()
        rate.sleep()
    rospy.spin()
