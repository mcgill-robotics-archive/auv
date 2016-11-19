#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Wrench, Vector3
from tf import TransformListener


class YawMaintainer():
    def __init__(self, desired_yaw):
        self.listener = TransformListener()
        self.thrust_pub = rospy.Publisher('controls/update', Wrench)
        self.desired_yaw = desired_yaw

        self.set_yaw(desired_yaw)

    def set_yaw(self, yaw):
        set_point_pub = rospy.Publisher('controls/set_point', Wrench)
        translation = Vector3(None, None, None)
        rotation = Vector3(None, None, yaw)
        if yaw is None:
            trans, rot = self.listener.lookupTransform(
                    'initial_horizon', 'robot', rospy.Time())
            rotation.z = rot[2]
            self.desired_yaw = rotation.z
        set_wrench = Wrench(translation, rotation)
        set_point_pub.publish(set_wrench)

    def update(self):
        trans, rot = self.listener.lookupTransform(
                    'initial_horizon', 'robot', rospy.Time())
        estimated_yaw = rot[2]
        yaw_error = self.desired_yaw - estimated_yaw
        translation = Vector3(None, None, None)
        rotation = Vector3(None, None, yaw_error)
        error_wrench = Wrench(translation, rotation)
        self.thrust_pub.publish(error_wrench)


if __name__ == '__main__':
    rospy.init_node('maintain_yaw')
    # TODO: investigate actionLiv server
    yaw_maintainer = YawMaintainer(1)
    timer = rospy.Timer(rospy.Duration(0.1), yaw_maintainer.update)
    rospy.on_shutdown(timer.shutdown)
    rospy.spin()
