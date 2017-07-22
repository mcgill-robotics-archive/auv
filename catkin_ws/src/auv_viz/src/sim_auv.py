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

        self.period = 0.1   # We know that controls publishes at this freq.
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

        self.drag_coeff_x = 22      # all drag coeffs together
        self.drag_coeff_y = 30      # all drag coeffs together
        self.drag_coeff_z = 20      # all drag coeffs together
        self.drag_coeff_theta = 8   # all drag coeffs together
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

        # Brodcast floating horizon.
        self.broadcaster.sendTransform(
            (0, 0, 0),
            (0, 0, 0, 1),
            rospy.Time.now(),
            "floating_horizon",
            self.map_frame
        )

        # Brodcast initial horizon, which is upside down compared to map.
        self.broadcaster.sendTransform(
            (0, 0, 0),
            quaternion_from_euler(np.pi, 0, 0),
            rospy.Time.now(),
            "initial_horizon",
            self.map_frame
        )

        # Publish the depth on a topic.
        self.depth_pub.publish(-self.current_pose.position.z)

        # Reset the twist to zero if we haven't gotten a message in too long.
        if (rospy.Time.now() - self.last_time).to_sec() > 2 * self.period:
            self.last_twist = Twist()

    def control_cb(self, msg):
        twist = self.wrench_to_twist(msg)

        # Integrate to update position.
        self.current_pose = self.updatePose(twist, self.current_pose)

        # Ensure that the robot doesn't go above the water surface.
        if self.current_pose.position.z > self.surface:
            self.current_pose.position.z = self.surface

        self.last_twist = twist
        self.last_time = rospy.Time.now()

    def wrench_to_twist(self, wrench):
        """Approximates the twist command associated with each drag.

        The robot's velocity is derived as follows:

            F_applied - F_drag = m * a = m * (v - v_0) / dt
            v = (F_applied - F_drag) * dt / m + v_0

        Drag is calculated as a function of velocity. Buoyancy is ignored for
        now, for simplicity. It is also assumed that the robot will never pitch
        or roll.

        Args:
            wrench: The wrench command sent to the robot.
        """
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

    def updatePose(self, twist, pose):
        """Updates the pose of the robot over a single period using the given
        twist.

        The motion model of the robot is simplified greatly. It is treated as
        an omnidirectional robot in the x-y plane, with added ability to move
        vertically in the z direction. The motion model in the x-y plane is:

            x_dot = dx / dt = v_x * cos(theta) - v_y * sin(theta)
            y_dot = dy / dt = v_y * cos(theta) + v_x * sin(theta)
            theta_dot = dtheta / dt = v_theta

        and in the z direction is:

            z_dot = dz / dt = v_z

        where the vs are applied velocities.

        Args:
            twist: The current twist of the robot.
            pose: The current pose of the robot.

        Returns:
            The updated pose of the robot.
        """
        shifted_pose = Pose()
        yaw = -euler_from_quaternion([pose.orientation.x,
                                      pose.orientation.y,
                                      pose.orientation.z,
                                      pose.orientation.w])[2]

        dx = self.period * (twist.linear.x * np.cos(yaw) - twist.linear.y * np.sin(yaw))
        dy = self.period * (twist.linear.y * np.cos(yaw) + twist.linear.x * np.sin(yaw))
        dz = self.period * twist.linear.z
        dtheta = self.period * twist.angular.z

        shifted_pose.position.x = pose.position.x + dx
        shifted_pose.position.y = pose.position.y - dy
        shifted_pose.position.z = pose.position.z - dz

        quat = quaternion_from_euler(0, 0, -normalize_angle(yaw + dtheta))
        shifted_pose.orientation.x = quat[0]
        shifted_pose.orientation.y = quat[1]
        shifted_pose.orientation.z = quat[2]
        shifted_pose.orientation.w = quat[3]

        return shifted_pose

    def drag(self, v, axis):
        """Calculates the drag force as a function of velocity.

        The drag coefficients are simplified to a single coefficient which is
        tuned. Drag in pitch and roll are not supported.

        Args:
            v: The velocity of the robot in the given axis.
            axis: The axis in which the robot is moving.

        Returns:
            The drag force.
        """
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
