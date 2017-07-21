#!/usr/bin/env python
import tf
import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes


class Buoy(object):

    def __init__(self):
        self.marker_pub = rospy.Publisher("sim/fake_buoy", Marker, queue_size=1)
        self.buoy_pub = rospy.Publisher("darknet_ros/bounding_boxes", BoundingBoxes, queue_size=1)

        self.buoy_radius = 0.2

        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.header.stamp = rospy.get_rostime()
        self.marker.type = Marker.SPHERE
        self.marker.scale.x = self.buoy_radius * 2
        self.marker.scale.y = self.buoy_radius * 2
        self.marker.scale.z = self.buoy_radius * 2
        self.marker.lifetime.secs = 0.1

        # Change this to change the position of the buoy relative to the origin.
        self.buoy_pos = (4.0, 1.0, -2.0)

        self.marker.color = ColorRGBA(1.0, 0, 0, 1.0)  # Red
        self.marker.pose.position.x = self.buoy_pos[0]
        self.marker.pose.position.y = self.buoy_pos[1]
        self.marker.pose.position.z = self.buoy_pos[2]

        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        # Projection matrix and image resolution, taken from camera calibration.
        self.P = [900.273804, 0.0, 651.730708, 0.0,
                  0.0, 955.260315, 448.241861, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        self.im_width = 1296
        self.im_height = 964

        # Start timers.
        self.timer = rospy.Timer(rospy.Duration(0.1), self.pub_buoy)
        self.br_timer = rospy.Timer(rospy.Duration(0.1), self.broadcast_buoy)

    def broadcast_buoy(self, _):
        # Create the tf.
        self.br.sendTransform(self.buoy_pos,
                              (0.0, 0.0, 0.0, 1.0),
                              rospy.Time.now(),
                              "buoy",
                              "map")

        # Publish the marker for visualization.
        self.marker.header.stamp = rospy.Time.now()
        self.marker_pub.publish(self.marker)

    def pub_buoy(self, _):
        # Get the transform between the buoy and the robot.
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("robot", "buoy", now, rospy.Duration(1.0))
            trans, rot = self.listener.lookupTransform("robot", "buoy", now)
        except Exception:
            rospy.logerr_throttle(30, "Can't get tranform between buoy and robot")
            return

        # Publish the message
        bbox = BoundingBox()

        # Create the bounding box
        # Real world y translates to x axis in pixels
        # Real world z translates to y axis in pixels
        # Origin is in the top left corner.
        bbox.Class = "buoy"
        bbox.probability = 1
        bbox.xmin = self.get_pixels(Point(trans[0], trans[1] - self.buoy_radius, trans[2]))
        bbox.ymin = self.get_pixels(Point(trans[0], trans[1], trans[2] - self.buoy_radius))
        bbox.xmax = self.get_pixels(Point(trans[0], trans[1] + self.buoy_radius, trans[2]))
        bbox.xmin = self.get_pixels(Point(trans[0], trans[1], trans[2] + self.buoy_radius))

        # If any of them were not in the frame, make this empty.
        if None in (bbox.xmin, bbox.xmax, bbox.ymin, bbox.ymax):
            bbox = BoundingBox()

        bboxes = BoundingBoxes()

        bboxes.boundingBoxes.append(bbox)
        self.buoy_pub.publish(bboxes)

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
        pixels.x = int((fx * point.y + Tx) / point.x + cx)
        pixels.y = int((fy * point.z + Ty) / point.x + cy)

        if pixels.x > self.im_width or pixels.y > self.im_height:
            return

        return pixels


if __name__ == '__main__':
    rospy.init_node("fake_lane")

    buoy = Buoy()

    rospy.spin()
