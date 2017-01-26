#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Wrench, Vector3
from std_msgs.msg import Float64
from PID import PID


class PoseController:
    def __init__(self):
        self.thrust_pub = rospy.Publisher(
                'controls/wrench', Wrench, queue_size=10)

        # initialize PIDs with gains from rosparams & add subscribers
        # translation
        self.surge_pid = PID(
                rospy.get_param("~kp_xPos"),
                rospy.get_param("~ki_xPos"),
                rospy.get_param("~kd_xPos"))
        rospy.Subscriber("controls/error/surge", Float64,
                         lambda data: self.surge_pid.update(data))
        self.sway_pid = PID(
                rospy.get_param("~kp_yPos"),
                rospy.get_param("~ki_yPos"),
                rospy.get_param("~kd_yPos"))
        rospy.Subscriber("controls/error/sway", Float64,
                         lambda data: self.sway_pid.update(data))
        self.heave_pid = PID(
                rospy.get_param("~kp_depth"),
                rospy.get_param("~ki_depth"),
                rospy.get_param("~kd_depth"))
        rospy.Subscriber("controls/error/heave", Float64,
                         lambda data: self.heave_pid.update(data))

        # rotation
        self.roll_pid = PID(
                rospy.get_param("~kp_roll"),
                rospy.get_param("~ki_roll"),
                rospy.get_param("~kd_roll"))
        rospy.Subscriber("controls/error/roll", Float64,
                         lambda data: self.roll_pid.update(data))
        self.pitch_pid = PID(
                rospy.get_param("~kp_pitch"),
                rospy.get_param("~ki_pitch"),
                rospy.get_param("~kd_pitch"))
        rospy.Subscriber("controls/error/pitch", Float64,
                         lambda data: self.pitch_pid.update(data))
        self.yaw_pid = PID(
                rospy.get_param("~kp_yaw"),
                rospy.get_param("~ki_yaw"),
                rospy.get_param("~kd_yaw"))
        rospy.Subscriber("controls/error/yaw", Float64,
                         lambda data: self.yaw_pid.update(data))

    def update(self, _):
        force = Vector3()
        force.x = self.surge_pid.output
        force.y = self.sway_pid.output
        force.z = self.heave_pid.output

        torque = Vector3()
        torque.x = self.roll_pid.output
        torque.y = self.pitch_pid.output
        torque.z = self.yaw_pid.output

        thrust_wrench = Wrench(force, torque)
        self.thrust_pub.publish(thrust_wrench)


if __name__ == '__main__':
    rospy.init_node('controls')

    pose_controller = PoseController()
    # TODO: experiment with referesh rate
    timer = rospy.Timer(rospy.Duration(0.1), pose_controller.update)
    rospy.on_shutdown(timer.shutdown)
    rospy.spin()
