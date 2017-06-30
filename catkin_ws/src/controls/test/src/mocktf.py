#! /usr/bin/env python

from tf.transformations import quaternion_from_euler
import rospy
import tf


def handle_pose(_):
    br = tf.TransformBroadcaster()
    br.sendTransform((0, 0, 1),
                     quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     'robot',
                     'floating_horizon')

if __name__ == '__main__':
    rospy.init_node('mock_tf_broadcaster')
    rospy.Timer(rospy.Duration(1), handle_pose)
    rospy.spin()
