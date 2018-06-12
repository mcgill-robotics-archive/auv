#!/usr/bin/env python

import rospy
import visualization_msgs.msg


class Mark:
    def __init__(self):
        self.marker = visualization_msgs.msg.Marker()

        self.marker.header.frame_id = "base_link"
        self.marker.header.stamp = rospy.Time.now()
        self.marker.ns = "my_namespace"
        self.marker.id = 0
        self.marker.type = 1  # 1 = CUBE
        self.marker.action = 0  # 0 = ADD
        self.marker.pose.position.x = 2.0
        self.marker.pose.position.y = 0.4
        self.marker.pose.position.z = 0.2
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.5
        self.marker.scale.y = 0.5
        self.marker.scale.z = 0.5
        self.marker.color.a = 1.0  # Don't forget to set the alpha
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 1.0


if __name__ == "__main__":
    rospy.init_node('marker_node')

    mark = Mark()
    pub = rospy.Publisher('/visualization_marker',
                          visualization_msgs.msg.Marker,
                          queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(mark.marker)
        rate.sleep()

rospy.spin()
