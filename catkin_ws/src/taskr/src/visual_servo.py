#!/usr/bin/env python
from actionlib import SimpleActionClient

from auv_msgs.msg import VisualServoAction, VisualServoGoal

from geometry_msgs.msg import Point, Vector3Stamped

from move import Move

import rospy

from std_msgs.msg import Float64

from tld_msgs.msg import BoundingBox

IMAGE_WIDTH = 1296
IMAGE_HEIGHT = 964
IMAGE_CENTER_X = IMAGE_WIDTH / 2
IMAGE_CENTER_Y = IMAGE_HEIGHT / 2

# TODO: Parametrize:
TARGET_WIDTH = 0.20
TARGET_HEIGHT = 0.20
DESIRED_DISTANCE_TO_TARGET = 1

PIXEL_THRESH_DONE = 800 * TARGET_WIDTH / DESIRED_DISTANCE_TO_TARGET
OPEN_LOOP_SURGE_DIST = 0.5


class VisualServo(object):
    def __init__(self, target):
        rospy.loginfo("Initializing VisualServo action with target {}"
                      .format(target))
        self.target = target
        self.controls_client = SimpleActionClient("controls_vservo",
                                                  VisualServoAction)
        self.controls_client.wait_for_server()

        # aimed x and y define the pixel on the image to which we aim the
        # center of the bounding box
        self.aimed_x = IMAGE_CENTER_X
        self.aimed_y = IMAGE_CENTER_Y
        self.target_width = TARGET_WIDTH
        self.target_height = TARGET_HEIGHT
        self.bb_size_width = 0

        self.models_path = rospy.get_param("~models_path")

        self.current_yaw = 0
        self.current_depth = 0

        self.pub = rospy.Publisher("target", Point, queue_size=10)
        self.sub_tracked_object = rospy.Subscriber(
            '/tld_tracked_object', BoundingBox, self.tracked_obj_callback)
        self.sub_yaw = rospy.Subscriber(
            '/robot_state', Vector3Stamped, self.yaw_callback)
        self.sub_depth = rospy.Subscriber(
            '/state_estimation/depth', Float64, self.depth_callback)

    def start(self, server, feedback_msg):
        rospy.loginfo("Starting VisualServo action")

        ctrl_goal = VisualServoGoal()
        ctrl_goal.cmd.target_frame_id = "target"
        ctrl_goal.cmd.yaw = self.current_yaw

        rospy.logdebug("Visual Servo Goal: {}".format(ctrl_goal))

        rospy.set_param('ros_tld_tracker_node/modelImportFile',
                        self.models_path + self.target)
        rospy.set_param('ros_tld_tracker_node/loadModel', True)

        rospy.logdebug("Visual Servoing")

        while True:
            self.controls_client.send_goal(ctrl_goal)

            # Check if we received preempt request from Taskr.
            if server.is_preempt_requested():
                rospy.loginfo("Visual Servo Preempted")
                # Send preempt request to Controls
                self.controls_client.cancel_goal()
                server.set_preempted()
                return

            rospy.loginfo("Waiting for results")
            self.controls_client.wait_for_result()
            rospy.loginfo("Successfully aimed at target!")

            if self.bb_size_width > PIXEL_THRESH_DONE:
                rospy.loginfo("Visual servo complete: target is in range")
                break
            else:
                # Surge forward before doing vservo again
                move_cmd = {"distance": OPEN_LOOP_SURGE_DIST,
                            "depth": self.current_depth,
                            "yaw": self.current_yaw,
                            "feedback": False}
                move_action = Move(move_cmd)
                move_action.start(server, feedback_msg)
                rospy.loginfo("Surged forward")
                rospy.loginfo("BoundingBox Size: %i / %i", self.bb_size_width,
                              PIXEL_THRESH_DONE)

        # Stop tracking by setting model file to empty
        rospy.set_param('ros_tld_tracker_node/modelImportFile', '')

    def tracked_obj_callback(self, box):
        if box.confidence > 0.50:
            self.bb_size_width = box.width
            self.last_frame_time = rospy.get_rostime()

            box_center_x = box.x + box.width / 2
            box_center_y = box.y + box.height / 2

            x_pixels_to_target = box_center_x - self.aimed_x
            y_pixels_to_target = box_center_y - self.aimed_y

            # e.g. 100 px to target center / 50 px for the tracked buoy * 0.2m
            # buoy width = 0.4 m to target center
            x_dist_to_target = (float(x_pixels_to_target) / float(box.width) *
                                self.target_width)
            y_dist_to_target = (float(y_pixels_to_target) / float(box.height) *
                                self.target_height)

            point = Point()
            point.x = 0.0  # TODO: get x (surge) distance based on size of bb
            point.y = x_dist_to_target
            point.z = y_dist_to_target
            self.pub.publish(point)
        else:
            # TODO: try to find it again (after some time?)
            point = Point()
            point.x = 0.0
            point.y = 0.0
            point.z = 0.0
            self.pub.publish(point)

    def yaw_callback(self, robot_state):
        self.current_yaw = robot_state.vector.z

    def depth_callback(self, depth):
        self.current_depth = depth
