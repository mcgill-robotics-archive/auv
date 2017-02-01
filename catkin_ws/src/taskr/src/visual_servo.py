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


class VisualServo(object):
    def __init__(self, target):
        rospy.loginfo("Initializing VisualServo action with target {}"
                      .format(target))
        self.target = target
        self.target_width = rospy.get_param(
            "~vservo/{}/width".format(target), default=0.2)
        self.target_height = rospy.get_param(
            "~vservo/{}/height".format(target), default=0.2)
        self.desired_distance_to_target = rospy.get_param(
            "~vservo/desired_distance_to_target", default=1)
        self.open_loop_surge_distance = rospy.get_param(
            "~vservo/open_loop_surge_distance", default=0.5)
        self.recovery_action_timeout = rospy.Duration.from_sec(
            rospy.get_param("~vservo/recovery_action_timeout", default=20))
        self.servo_action_timeout = rospy.Duration.from_sec(
            rospy.get_param("~vservo/servo_action_timeout", default=15))
        self.end_action_timeout = rospy.Duration.from_sec(
            rospy.get_param("~vservo/end_action_timeout", default=60))
        self.recovery_half_dist_init = rospy.get_param(
            "~vservo/recovery_half_dist_init", default=0.5)
        self.recovery_max_tries = rospy.get_param(
            "~vservo/recovery_max_tries", default=3)

        self.recovery_half_dist = self.recovery_half_dist_init

        # The formula below was tuned by checking the pixel size of the 0.2m
        # wide buoy at 0.5, 1, 2, and 3m from the camera.
        # The pixel size is roughly inversely proportional to the distance.
        # At 1m, the 0.2m wide buoy is ~160 pixels on camera,
        # hence the factor of 160/0.2 = 800
        self.pixel_thresh_done = (800 * self.target_width /
                                  self.desired_distance_to_target)

        self.controls_client = SimpleActionClient("controls_vservo",
                                                  VisualServoAction)
        self.controls_client.wait_for_server()

        # aimed x and y define the pixel on the image to which we aim the
        # center of the bounding box
        self.aimed_x = IMAGE_CENTER_X
        self.aimed_y = IMAGE_CENTER_Y

        self.last_frame_time = rospy.get_rostime()
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

        recovery_tries = 0

        while not rospy.is_shutdown():
            if (rospy.get_rostime() - self.last_frame_time >
                    self.end_action_timeout):
                rospy.logwarn("Target lost, moving on")

                # Surge forward as last resort
                move_cmd = {"distance": 3 * self.open_loop_surge_distance}
                rospy.loginfo(move_cmd)
                move_action = Move(move_cmd)
                move_action.start(server, feedback_msg)
                rospy.loginfo("Surged forward")
                break

            self.controls_client.send_goal(ctrl_goal)

            # Check if we received preempt request from Taskr.
            if server.is_preempt_requested():
                rospy.loginfo("Visual Servo Preempted")
                # Send preempt request to Controls
                self.controls_client.cancel_goal()
                server.set_preempted()
                return

            rospy.loginfo("Waiting for results")
            result = self.controls_client.wait_for_result(
                self.servo_action_timeout)

            if result:  # Successfully centered
                rospy.loginfo("Successfully aimed at target!")

                if self.bb_size_width > self.pixel_thresh_done:
                    rospy.loginfo("Visual servo complete: target is in range")
                    break
                else:
                    # Surge forward before doing vservo again
                    move_cmd = {"distance": self.open_loop_surge_distance,
                                "depth": self.current_depth,
                                "yaw": self.current_yaw,
                                "feedback": False}
                    rospy.loginfo(move_cmd)
                    move_action = Move(move_cmd)
                    move_action.start(server, feedback_msg)
                    rospy.loginfo("Surged forward")
                    rospy.loginfo("BoundingBox Size: %i / %i",
                                  self.bb_size_width, self.pixel_thresh_done)
            else:  # Did not center on time
                rospy.logwarn("Target servo timed out")
                if (rospy.get_rostime() - self.last_frame_time <
                        self.recovery_action_timeout):
                    rospy.loginfo("Not time to end yet, try again")
                    continue

                if recovery_tries >= self.recovery_max_tries:
                    rospy.logwarn("Tried and failed. Moving on.")
                    return_move_cmd = {"distance": -1 *
                                       self.recovery_half_dist_init,
                                       "sway": True}
                    rospy.loginfo(return_move_cmd)
                    recovery_move_action = Move(return_move_cmd)
                    recovery_move_action.start(server, feedback_msg)
                    break

                if recovery_tries == 0:  # half step to the left
                    self.recovery_dist = -1 * self.recovery_half_dist_init
                elif recovery_tries == 1:  # whole step to the opposite direction
                    self.recovery_dist *= -2
                else:  # whole step to the opposite direction
                    self.recovery_dist *= -1

                recovery_move_cmd = {"distance": self.recovery_dist,
                                     "sway": True}

                rospy.loginfo(recovery_move_cmd)
                recovery_move_action = Move(recovery_move_cmd)
                recovery_move_action.start(server, feedback_msg)

                recovery_surge_cmd = {"distance": 0.7}
                rospy.loginfo(recovery_surge_cmd)
                recovery_surge_action = Move(recovery_surge_cmd)
                recovery_surge_action.start(server, feedback_msg)

                recovery_tries += 1

        # Stop tracking by setting model file to empty
        rospy.set_param('ros_tld_tracker_node/modelImportFile', '')

    def tracked_obj_callback(self, box):
        if box.confidence > 0.50:
            self.last_frame_time = rospy.get_rostime()
            self.bb_size_width = box.width

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
            point = Point()
            point.x = 0.0
            point.y = 0.0
            point.z = 0.0
            self.pub.publish(point)

    def yaw_callback(self, robot_state):
        self.current_yaw = robot_state.vector.z

    def depth_callback(self, depth):
        self.current_depth = depth.data
