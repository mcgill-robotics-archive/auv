#!/usr/bin/env python


import rospy
import tf
from geometry_msgs.msg import Wrench, Vector3
from tf import TransformListener


class DepthMaintainer():
    '''
    Publishes error between desired depth and current depth to PID controller,
    i.e. desired_depth - current_depth = error
    '''
    def __init__(self, desired_depth=None):
        self.listener = TransformListener()
        self.thrust_pub = rospy.Publisher(
            'controls/update', Wrench, queue_size=10)
        self.desired_depth = self._set_depth(desired_depth)

    def _get_current_depth(self):
        '''
        Handles the transform lookup to get the current depth
        '''
        while not rospy.is_shutdown():
            try:
                trans, rot = self.listener.lookupTransform(
                        '/floating_horizon', '/robot', rospy.Time())
                return trans[2]
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException):
                continue
        raise rospy.ROSInterruptException()

    def _set_depth(self, depth):
        '''
        Sets the current depth to the set point if one isn't specified
        '''
        return depth if depth is not None else self._get_current_depth()

    def update(self, _):
        '''
        updates the PID controller with the current error
        from the desired depth
        '''
        estimated_depth = self._get_current_depth()
        depth_error = self.desired_depth - estimated_depth
        translation = Vector3(None, None, depth_error)
        rotation = Vector3(None, None, None)
        error_wrench = Wrench(translation, rotation)
        self.thrust_pub.publish(error_wrench)


if __name__ == '__main__':
    rospy.init_node('maintain_depth')

    desired_depth = None
    if rospy.has_param('~desired_depth'):
        desired_depth = rospy.get_param('~desired_depth')
    depth_maintainer = DepthMaintainer(desired_depth)

    timer = rospy.Timer(rospy.Duration(0.1), depth_maintainer.update)
    rospy.on_shutdown(timer.shutdown)
    rospy.spin()
