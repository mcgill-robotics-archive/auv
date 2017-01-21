#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Wrench, Vector3
from controls.msg import Pose
from PID import PID


class Controller:
    def __init__(self):
        self.thrust_pub = rospy.Publisher(
                'controls/wrench', Wrench, queue_size=10)

        # initialize PIDs with gains from rosparams
        self.surge_pid = PID(
                rospy.get_param("~kp_xPos"),
                rospy.get_param("~ki_xPos"),
                rospy.get_param("~kd_xPos"))
        self.sway_pid = PID(
                rospy.get_param("~kp_yPos"),
                rospy.get_param("~ki_yPos"),
                rospy.get_param("~kd_yPos"))
        self.heave_pid = PID(
                rospy.get_param("~kp_depth"),
                rospy.get_param("~ki_depth"),
                rospy.get_param("~kd_depth"))

        self.roll_pid = PID(
                rospy.get_param("~kp_roll"),
                rospy.get_param("~ki_roll"),
                rospy.get_param("~kd_roll"))
        self.pitch_pid = PID(
                rospy.get_param("~kp_pitch"),
                rospy.get_param("~ki_pitch"),
                rospy.get_param("~kd_pitch"))
        self.yaw_pid = PID(
                rospy.get_param("~kp_yaw"),
                rospy.get_param("~ki_yaw"),
                rospy.get_param("~kd_yaw"))

    def update(self, data):
        """
        Update each DoF when new data is given
        """
        trans = Vector3()
        trans.x = self.surge_pid.update(data.translation.x)
        trans.y = self.sway_pid.update(data.translation.y)
        trans.z = self.heave_pid.update(data.translation.z)

        rot = Vector3()
        rot.x = self.roll_pid.update(data.rotation.x)
        rot.y = self.pitch_pid.update(data.rotation.y)
        rot.z = self.yaw_pid.update(data.rotation.z)

        self.thrust_pub.publish(Pose(trans, rot))


if __name__ == '__main__':
    rospy.init_node('controls')
    controller = Controller()
    update_sub = rospy.Subscriber(
            'controls/error', Pose, controller.update)
    rospy.spin()
