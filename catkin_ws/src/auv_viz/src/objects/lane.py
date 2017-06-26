#!/usr/bin/env python
import tf
import rospy
import numpy as np
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point


class Lane(object):

    def __init__(self):
        self.marker_pub = rospy.Publisher("sim/fake_lane", Marker, queue_size=1)
        self.lane_pub = rospy.Publisher("state_estimation/lane", PolygonStamped, queue_size=1)

        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.header.stamp = rospy.get_rostime()
        self.marker.type = Marker.CUBE
        self.marker.scale.x = 0.8
        self.marker.scale.y = 0.2
        self.marker.scale.z = 0.1
        self.marker.lifetime.secs = 0.1

        self.lane_pos = (2.0, 0.0, -5.0)

        self.marker.color = ColorRGBA(1.0, 0.5, 0, 1.0)
        self.marker.pose.position.x = self.lane_pos[0]
        self.marker.pose.position.y = self.lane_pos[1]
        self.marker.pose.position.z = self.lane_pos[2]

        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        self.polygon_pts = [Point(0.4, 0.1, self.lane_pos[2]),
                            Point(-0.4, 0.1, self.lane_pos[2]),
                            Point(-0.4, -0.1, self.lane_pos[2]),
                            Point(0.4, -0.1, self.lane_pos[2])]
        # Place holder in case you want to change yaw.
        self.polygon_pts = self.transform_polygon(self.polygon_pts, 0, 0, 0)

        # Projection matrix and image resolution, taken from camera calibration.
        self.P = [900.273804, 0.0, 651.730708, 0.0,
                  0.0, 955.260315, 448.241861, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        self.im_width = 1296
        self.im_height = 964

        # Start timers.
        self.timer = rospy.Timer(rospy.Duration(0.1), self.pub_lane)
        self.br_timer = rospy.Timer(rospy.Duration(0.1), self.broadcast_lane)

    def broadcast_lane(self, _):
        # Create the tf.
        self.br.sendTransform(self.lane_pos,
                              (0.0, 0.0, 0.0, 1.0),
                              rospy.Time.now(),
                              "lane",
                              "map")

        # Publish the marker for visualization.
        self.marker.header.stamp = rospy.Time.now()
        self.marker_pub.publish(self.marker)

    def pub_lane(self, _):
        # Get the transform between the lane and the robot.
        try:
            self.listener.waitForTransform("lane", "base_link", rospy.Time.now(), rospy.Duration(1.0))
            trans, rot = self.listener.lookupTransform("lane", "base_link", rospy.Time())
        except Exception:
            rospy.logerr_throttle(30, "Can't get tranform between lane and base_link")
            return

        roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)

        # Rotate the polygon by the yaw, and shift by the translation.
        transformed = self.transform_polygon(self.polygon_pts, trans[0], trans[1], yaw)

        # Create polygon to publish.
        polygon = PolygonStamped()
        polygon.header.frame_id = "robot"
        polygon.header.stamp = rospy.Time.now()

        # Add the points in pixels.
        for pt in transformed:
            pixels = self.get_pixels(pt)
            # If the pixels exceeded the image size, empty the list and publish
            # an empty array.
            if pixels:
                polygon.polygon.points.append(pixels)
            else:
                polygon.polygon.points = []
                break

        self.lane_pub.publish(polygon)

    def get_pixels(self, point):
        """Project a 3D point into pixel coordinates.

        The projection matrix is:
                [fx'  0  cx' Tx]
            P = [ 0  fy' cy' Ty]
                [ 0   0   1   0]
        and the pixel coordinates are:
            u = (fx * pt.x + Tx) / pt.z + cx
            v = (fy * pt.y + Ty) / pt.z + cy;
        """
        if point.z == 0:
            return

        fx = self.P[0 + 0 * 4]
        fy = self.P[1 + 1 * 4]
        cx = self.P[2 + 0 * 4]
        cy = self.P[2 + 1 * 4]
        Tx = self.P[3 + 0 * 4]
        Ty = self.P[3 + 1 * 4]

        pixels = Point()
        pixels.x = (fx * point.x + Tx) / (-point.z) + cx
        pixels.y = (fy * point.y + Ty) / (-point.z) + cy

        if pixels.x > self.im_width or pixels.y > self.im_height:
            return None

        return pixels

    def transform_polygon(self, poly, x, y, yaw):
        """Should be a utility function. Rotate a polygon as follows:

            P_new = P * R

        where P and P_new are of the form [[x1, x2 ... xn], [y1, y2 ... yn]]
        and R is [[cos(yaw), -sin(yaw)], [sin(yaw), cos(yaw)]]."""
        transformed = []
        for ele in poly:
            pt = Point()
            pt.x = ele.x * np.cos(yaw) - ele.y * np.sin(yaw)
            pt.y = ele.y * np.sin(yaw) + ele.y * np.cos(yaw)
            pt.z = self.lane_pos[2]
            transformed.append(pt)

        for ele in transformed:
            ele.x += x
            ele.y += y

        return transformed


if __name__ == '__main__':
    rospy.init_node("fake_lane")

    lane = Lane()

    rospy.spin()
