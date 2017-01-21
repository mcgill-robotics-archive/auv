#!/usr/bin/env python


import rospy
from math import pi
from geometry_msgs.msg import Vector3
from controls.msg import Pose
from tf import TransformListener
from tf.transformations import euler_from_quaternion
import tf


class YawMaintainer():
    def __init__(self, desired_yaw):
        self.listener = TransformListener()
        self.thrust_pub = rospy.Publisher('controls/error', Pose, queue_size=1)
        self.desired_yaw = self.set_yaw(desired_yaw)

    def set_yaw(self, yaw):
        if yaw is None:
            yaw = self.get_current_yaw()
        return yaw

    def update(self, _):
        estimated_yaw = self.get_current_yaw()
        yaw_error = self.desired_yaw - estimated_yaw
        yaw_error = self.normalize_angle(yaw_error)
        translation = Vector3(None, None, None)
        rotation = Vector3(None, None, yaw_error)
        error = Pose(translation, rotation)
        self.thrust_pub.publish(error)

    def normalize_angle(self, angle, max_angle=pi):
        # Returns angle between -max_angle and max_angle
        angle = angle % (2 * pi)
        if angle > pi:
            angle -= 2 * pi
        return angle

    def get_current_yaw(self):
        while not rospy.is_shutdown():
            try:
                trans, rot = self.listener.lookupTransform(
                    'initial_horizon', 'robot', rospy.Time())
                (roll, pitch, yaw) = euler_from_quaternion(rot)
                return yaw
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException):
                continue
        raise rospy.ROSInterruptException()


if __name__ == '__main__':
    rospy.init_node('maintain_yaw')
    # TODO: investigate actionLib server

    desired_yaw = None
    if rospy.has_param('~desired_yaw'):
        desired_yaw = rospy.get_param('~desired_yaw')
    yaw_maintainer = YawMaintainer(desired_yaw)

    timer = rospy.Timer(rospy.Duration(0.1), yaw_maintainer.update)
    rospy.on_shutdown(timer.shutdown)
    rospy.spin()
