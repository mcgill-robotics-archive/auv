#!/usr/bin/env python
import tf
import rospy
import numpy as np
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped
from controls.utils import normalize_angle


class Pinger(object):

    def __init__(self):
        self.marker_pub = rospy.Publisher("sim/fake_pinger", Marker, queue_size=1)
        self.pinger_pub = rospy.Publisher("hydrophones/heading", Float64, queue_size=1)

        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.header.stamp = rospy.get_rostime()
        self.marker.type = Marker.CYLINDER
        self.marker.scale.x = 0.05
        self.marker.scale.y = 0.05
        self.marker.scale.z = 0.2
        self.marker.lifetime.secs = 0.1

        self.pinger_pos = (6.0, 5.0, -5.0)

        self.marker.color = ColorRGBA(0, 0, 0, 1.0)
        self.marker.pose.position.x = self.pinger_pos[0]
        self.marker.pose.position.y = self.pinger_pos[1]
        self.marker.pose.position.z = self.pinger_pos[2]

        quat = quaternion_from_euler(np.pi / 2, 0, 0)

        self.marker.pose.orientation.x = quat[0]
        self.marker.pose.orientation.y = quat[1]
        self.marker.pose.orientation.z = quat[2]
        self.marker.pose.orientation.w = quat[3]

        self.yaw = None

        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        # Start timers.
        self.br_timer = rospy.Timer(rospy.Duration(0.1), self.broadcast_pinger)
        self.listener.waitForTransform("robot", "pinger", rospy.Time(0), rospy.Duration(1.0))

        self.sub = rospy.Subscriber("robot_state", Vector3Stamped, self.attitude_cb, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(2), self.pub_pinger)

    def broadcast_pinger(self, _):
        # Create the tf.
        self.br.sendTransform(self.pinger_pos,
                              (0.0, 0.0, 0.0, 1.0),
                              rospy.Time.now(),
                              "pinger",
                              "map")

        # Publish the marker for visualization.
        self.marker.header.stamp = rospy.Time.now()
        self.marker_pub.publish(self.marker)

    def pub_pinger(self, _):
        # Get the transform between the lane and the robot.
        if self.yaw is None:
            return

        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("robot", "initial_horizon", now, rospy.Duration(1.0))
            trans, rot = self.listener.lookupTransform("robot", "initial_horizon", now)
        except Exception:
            rospy.logerr_throttle(30, "Can't get tranform between pinger and robot")
            return

        relative_x = self.pinger_pos[0] - trans[0]
        relative_y = -self.pinger_pos[1] - trans[1]

        self.pinger_pub.publish(normalize_angle(np.arctan2(relative_y, relative_x) - self.yaw))

    def attitude_cb(self, msg):
        self.yaw = msg.vector.z


if __name__ == '__main__':
    rospy.init_node("fake_lane")

    lane = Pinger()

    rospy.spin()
