#!/usr/bin/env python

import rospy
import tf
import numpy as np
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Vector3
from tf.transformations import quaternion_from_euler


class FakeAUV(object):

    def __init__(self):
        self.broadcaster = tf.TransformBroadcaster()
        self.robot_frame = "robot"
        self.map_frame = "map"

        self.window = 2
        self.period = 0.1  # We know that controls publishes at this freq.
        self.vel_history = [Vector3(), Vector3()]
        self.ang_vel_history = [Vector3(), Vector3()]

        self.surface = 0.0  # Surface of the water.

        self.control_sub = rospy.Subscriber("controls/wrench", Wrench, self.control_cb, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(self.period), self.broadcast)

        self.current_pos = Vector3()
        self.current_angle = Vector3()

        self.m = 35                 # mass - kg
        self.g = 9.81               # m / s2
        self.V = 40                 # volume - L
        self.V = self.V * 0.001     # m^3
        self.rho = 1000             # kg/m^3

        self.drag_coeff_x = 10      # all drag coeffs together
        self.drag_coeff_y = 30      # all drag coeffs together
        self.drag_coeff_z = 20      # all drag coeffs together
        self.drag_coeff_theta = 3   # all drag coeffs together
        self.rot_coeff = 0.1        # r / I

        # We'll ignore buoyancy for now, because we lack a water surface (avoid floating up forever)
        # TODO - Jana: Add a water surface by setting z velocity to zero above a surface level/.
        self.Fg = self.m * self.g             # Grafivational force
        self.Fb = self.V * self.rho * self.g  # Buoyancy force

    def broadcast(self, _):
        # Turn the distances into the correct form.
        quaternion = quaternion_from_euler(self.current_angle.x, self.current_angle.y, self.current_angle.z)
        position = (self.current_pos.x, -self.current_pos.y, -self.current_pos.z)

        # Brodcast the transform.
        self.broadcaster.sendTransform(
            position,
            quaternion,
            rospy.Time.now(),
            self.robot_frame,
            self.map_frame
        )

    def control_cb(self, msg):
        vel, ang_vel = self.wrench_to_twist(msg)

        # Add the velocity to the history.
        if self.vel_history >= self.window:
            self.vel_history = self.vel_history[1:]
        self.vel_history.append(vel)

        if self.ang_vel_history >= self.window:
            self.ang_vel_history = self.ang_vel_history[1:]
        self.ang_vel_history.append(ang_vel)

        # Integrate to get distance change.
        self.current_pos = add(self.current_pos, self.integrateAll(self.vel_history))
        self.current_angle = add(self.current_angle, self.integrateAll(self.ang_vel_history))

        if self.current_pos.z < self.surface:
            self.current_pos.z = self.surface

    def wrench_to_twist(self, wrench):
        vel = Vector3()
        ang_vel = Vector3()

        # Do force to velocity conversion. Assume the only angular velocity is yaw.
        vel.x = (wrench.force.x -
                 self.drag(self.vel_history[0].x, "x")) * self.period / self.m + self.vel_history[0].x
        vel.y = (wrench.force.y -
                 self.drag(self.vel_history[0].y, "y")) * self.period / self.m + self.vel_history[0].y
        vel.z = (wrench.force.z -
                 self.drag(self.vel_history[0].z, "z")) * self.period / self.m + self.vel_history[0].z

        ang_vel.z = ((wrench.torque.z - self.drag(self.ang_vel_history[0].z, "theta")) *
                     self.period * self.rot_coeff) + self.ang_vel_history[0].z

        return vel, ang_vel

    def integrateAll(self, history):
        x = []
        y = []
        z = []

        for vec in history:
            x.append(vec.x)
            y.append(vec.y)
            z.append(vec.z)

        dist = Vector3()

        dist.x = np.trapz(x, dx=self.period)
        dist.y = np.trapz(y, dx=self.period)
        dist.z = np.trapz(z, dx=self.period)

        return dist

    def drag(self, v, axis):
        if axis == "x":
            return self.drag_coeff_x * v
        if axis == "y":
            return self.drag_coeff_y * v
        if axis == "z":
            return self.drag_coeff_z * v
        if axis == "theta":
            return self.drag_coeff_theta * v
        rospy.logerr("Provide a valid axis. You provided {}.".format(axis))


def add(vec1, vec2):
    result = Vector3(vec1.x + vec2.x,
                     vec1.y + vec2.y,
                     vec1.z + vec2.z)
    return result


if __name__ == '__main__':
    rospy.init_node("auv_rviz_sim")
    auv = FakeAUV()

    rospy.spin()
