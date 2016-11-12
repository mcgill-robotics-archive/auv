#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Wrench, Vector3
from PID import PID


class Controller:
    def __init__(self):
        self.thrust_pub = rospy.Publisher('controls/wrench', Wrench)

        # TODO: Add gains from rosparams like in scripts/controls
        self.surge_pid = PID()
        self.sway_pid = PID()
        self.heave_pid = PID()

        self.roll_pid = PID()
        self.pitch_pid = PID()
        self.yaw_pid = PID()

    def update(self, data):
        force = Vector3()
        force.x = self.sway_pid.update(data.force.x)
        force.y = self.surge_pid.update(data.force.y)
        force.z = self.heave_pid.update(data.force.z)

        torque = Vector3()
        torque.x = self.pitch_pid.update(data.torque.x)
        torque.y = self.roll_pid.update(data.torque.y)
        torque.z = self.yaw_pid.update(data.torque.z)

        self.thrust_pub.publish(Wrench(force, torque))

    def set_point(self, data):
        force = Vector3()
        force.x = self.sway_pid.setPoint(data.force.x)
        force.y = self.surge_pid.setPoint(data.force.y)
        force.z = self.heave_pid.setPoint(data.force.z)

        torque = Vector3()
        torque.x = self.pitch_pid.setPoint(data.torque.x)
        torque.y = self.roll_pid.setPoint(data.torque.y)
        torque.z = self.yaw_pid.setPoint(data.torque.z)


if __name__ == '__main__':
    rospy.init_node('controls')
    controller = Controller()
    update_sub = rospy.Subscriber(
            'controls/update', Wrench, controller.update)
    setpoint_sub = rospy.Subscriber(
            'controls/set_point', Wrench, controller.setpoint)
    rospy.spin()
