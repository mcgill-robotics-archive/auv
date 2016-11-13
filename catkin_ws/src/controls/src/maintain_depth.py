#!/usr/bin/env python


import rospy
import tf
from geometry_msgs.msg import Wrench, Vector3
from tf import TransformListener


class DepthMaintainer():
    def __init__(self, desired_depth=None):
        self.listener = TransformListener()
        self.thrust_pub = rospy.Publisher('controls/update', Wrench)
        self.desired_depth = desired_depth

        self.set_depth(desired_depth)

    def get_current_depth(self):
        while not rospy.is_shutdown():
            try:
                trans, rot = self.listener.lookupTransform(
                        '/floating_horizon', '/robot', rospy.Time())
                return trans[2]
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException):
                continue

    def set_depth(self, depth):
        set_point_pub = rospy.Publisher('controls/set_point', Wrench)
        translation = Vector3(None, None, depth)
        rotation = Vector3(None, None, None)
        if depth is None:
            translation.z = self.get_current_depth()
            self.desired_depth = translation.z
        set_wrench = Wrench(translation, rotation)
        set_point_pub.publish(set_wrench)

    def update(self):
        estimated_depth = self.get_current_depth()
        depth_error = self.desired_depth - estimated_depth
        translation = Vector3(None, None, depth_error)
        rotation = Vector3(None, None, None)
        error_wrench = Wrench(translation, rotation)
        self.thrust_pub.publish(error_wrench)


if __name__ == '__main__':
    rospy.init_node('maintain_depth')
    depth_maintainer = DepthMaintainer()
    timer = rospy.Timer(rospy.Duration(0.1), depth_maintainer.update)
    rospy.on_shutdown(timer.shutdown)
    rospy.spin()
