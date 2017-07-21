#!/usr/bin/env python

import rospy
import tf
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Pose, Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from controls.utils import normalize_angle


class FakeAUV(object):

    def __init__(self):
        self.broadcaster = tf.TransformBroadcaster()
        self.robot_frame = "robot"
        self.map_frame = "map"

        self.window = 2
        self.period = 0.1  # We know that controls publishes at this freq.
        # self.vel_history = [Vector3(), Vector3()]
        # self.ang_vel_history = [Vector3(), Vector3()]

        self.surface = 0.0  # Surface of the water.

        self.current_pose = Pose()
        self.current_pose.orientation.w = 1
        self.last_twist = Twist()

        self.last_time = rospy.Time.now()

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

        # We'll ignore these for now for simplicity.
        self.Fg = self.m * self.g             # Grafivational force
        self.Fb = self.V * self.rho * self.g  # Buoyancy force

        self.depth_pub = rospy.Publisher("state_estimation/depth", Float64, queue_size=1)
        self.control_sub = rospy.Subscriber("controls/wrench", Wrench, self.control_cb, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(self.period), self.broadcast)

    def broadcast(self, _):
        # Turn the distances into the correct form.
        quaternion = (self.current_pose.orientation.x,
                      self.current_pose.orientation.y,
                      self.current_pose.orientation.z,
                      self.current_pose.orientation.w)
        position = (self.current_pose.position.x,
                    self.current_pose.position.y,
                    self.current_pose.position.z)

        # Brodcast the transform.
        self.broadcaster.sendTransform(
            position,
            quaternion,
            rospy.Time.now(),
            self.robot_frame,
            self.map_frame
        )

        # Brodcast floating horizon and initial horizon.
        self.broadcaster.sendTransform(
            (0, 0, 0),
            (0, 0, 0, 1),
            rospy.Time.now(),
            "floating_horizon",
            self.map_frame
        )

        # Brodcast floating horizon and initial horizon.
        self.broadcaster.sendTransform(
            (0, 0, 0),
            quaternion_from_euler(np.pi, 0, 0),
            rospy.Time.now(),
            "initial_horizon",
            self.map_frame
        )

        # Publish the depth on a topic as well.
        self.depth_pub.publish(self.current_pose.position.z)

        if (rospy.Time.now() - self.last_time).to_sec() > 2 * self.period:
            self.last_twist = Twist()

    def control_cb(self, msg):
        twist = self.wrench_to_twist(msg)

        # Integrate to get distance change.
        self.integrateAll(twist)
        # self.current_pos = add(self.current_pos, self.integrateAll(self.vel_history))
        # self.current_angle = add(self.current_angle, self.integrateAll(self.ang_vel_history))

        if self.current_pose.position.z < self.surface:
            self.current_pose.position.z = self.surface

        self.last_twist = twist
        self.last_time = rospy.Time.now()

    def wrench_to_twist(self, wrench):
        twist = Twist()

        # Do force to velocity conversion. Assume the only angular velocity is yaw.
        twist.linear.x = (wrench.force.x -
                          self.drag(self.last_twist.linear.x, "x")) * self.period / self.m + self.last_twist.linear.x
        twist.linear.y = (wrench.force.y -
                          self.drag(self.last_twist.linear.y, "y")) * self.period / self.m + self.last_twist.linear.y
        twist.linear.z = (wrench.force.z -
                          self.drag(self.last_twist.linear.z, "z")) * self.period / self.m + self.last_twist.linear.z

        twist.angular.z = ((wrench.torque.z - self.drag(self.last_twist.angular.z, "theta")) *
                           self.period * self.rot_coeff) + self.last_twist.angular.z

        return twist

    def integrateAll(self, twist):
        yaw = -euler_from_quaternion([self.current_pose.orientation.x,
                                      self.current_pose.orientation.y,
                                      self.current_pose.orientation.z,
                                      self.current_pose.orientation.w])[2]

        dx = self.period * (twist.linear.x * np.cos(yaw) - twist.linear.y * np.sin(yaw))
        dy = self.period * (twist.linear.y * np.cos(yaw) + twist.linear.x * np.sin(yaw))
        dtheta = self.period * twist.angular.z

        dz = self.period * twist.linear.z

        self.current_pose.position.x = self.current_pose.position.x + dx
        self.current_pose.position.y = -self.current_pose.position.y + dy
        self.current_pose.position.z = -self.current_pose.position.z + dz

        self.current_pose.position.z *= -1
        self.current_pose.position.y *= -1

        quat = quaternion_from_euler(0, 0, -normalize_angle(yaw + dtheta))
        self.current_pose.orientation.x = quat[0]
        self.current_pose.orientation.y = quat[1]
        self.current_pose.orientation.z = quat[2]
        self.current_pose.orientation.w = quat[3]

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


if __name__ == '__main__':
    rospy.init_node("auv_rviz_sim")
    auv = FakeAUV()

    rospy.spin()
