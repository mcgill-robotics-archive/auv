#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Wrench, Vector3
from tf import TransformListener


class DepthMaintainer():
    def __init__(self, desired_depth):
        self.listener = TransformListener()
        self.thrust_pub = rospy.Publisher('controls/update', Wrench)
        self.desired_depth = desired_depth

        self.set_depth(desired_depth)

    def set_depth(self, depth):
        set_point_pub = rospy.Publisher('controls/set_point', Wrench)
        translation = Vector3(None, depth, None)
        rotation = Vector3(None, None, None)
        set_wrench = Wrench(translation, rotation)
        set_point_pub.publish(set_wrench)

    def update(self):
        trans, rot = self.listener.lookupTransform(
                '/floating_horizon', '/robot', rospy.Time())
        estimated_depth = trans[2]
        depth_error = self.desired_depth - estimated_depth
        translation = Vector3(None, depth_error, None)
        rotation = Vector3(None, None, None)
        error_wrench = Wrench(translation, rotation)
        self.thrust_pub.publish(error_wrench)


if __name__ == '__main__':
    rospy.init_node('maintain_depth')
    # TODO: figure out how to initialize the depth desired
    depth_maintainer = DepthMaintainer(1)
    sub = rospy.Subscriber('robot_state', depth_maintainer.update)
    rospy.spin()
