#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Wrench, Vector3
from PID import PID


class Controller:
    def __init__(self):
        self.thrust_pub = rospy.Publisher(
                'controls/wrench', Wrench, queue_size=10)

        # initialize PIDs with gains from rosparams
        self.surge_pid = PID(
                rospy.get_param("~kp_yPos"),
                rospy.get_param("~ki_yPos"),
                rospy.get_param("~kd_yPos"))
        self.sway_pid = PID(
                rospy.get_param("~kp_xPos"),
                rospy.get_param("~ki_xPos"),
                rospy.get_param("~kd_xPos"))
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
        force = Vector3()
        force.x = self.surge_pid.update(data.force.x)
        force.y = self.sway_pid.update(data.force.y)
        force.z = self.heave_pid.update(data.force.z)

        torque = Vector3()
        torque.x = self.roll_pid.update(data.torque.x)
        torque.y = self.pitch_pid.update(data.torque.y)
        torque.z = self.yaw_pid.update(data.torque.z)

        self.thrust_pub.publish(Wrench(force, torque))


if __name__ == '__main__':
    rospy.init_node('controls')
    controller = Controller()
    update_sub = rospy.Subscriber(
            'controls/update', Wrench, controller.update)
    rospy.spin()
