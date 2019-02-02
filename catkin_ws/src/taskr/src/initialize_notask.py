#!/usr/bin/env python
import rospy
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from blinky.srv import UpdatePlannerLights
from auv_msgs.msg import InitializeHorizonAction, InitializeHorizonGoal
from geometry_msgs.msg import Wrench


class InitializerNoTask(object):
    def __init__(self, times):
        self.countdown = rospy.Duration(times["countdown"])
        self.drift_check = (rospy.Duration(times["drift_check"])
                            if "drift_check" in times else rospy.Duration(2))
        self.controls_pub = rospy.Publisher('controls/wrench',
                                            Wrench,
                                            queue_size=1)
        self.initialize_client = SimpleActionClient("initialize_horizon", InitializeHorizonAction)
        self.initialize_client.wait_for_server()
        print("wait for server done")

    def init_thrusters(self):
        zero_msg = Wrench()
        self.controls_pub.publish(zero_msg)

    def start(self):
        initialize_goal = InitializeHorizonGoal()
        initialize_goal.countdown = self.countdown
        initialize_goal.drift_check = self.drift_check

        self.initialize_client.send_goal(initialize_goal)

        #feedback_msg.is_done = False  # Not super useful feedback.
        #server.publish_feedback(feedback_msg)

        self.initialize_client.wait_for_result()
        print("wait for result done")
        if self.initialize_client.get_state() == GoalStatus.ABORTED:
            # Failed to initialize. Print panic lights.
            print("Failed to initialize")
            try:
                blinky_proxy = rospy.ServiceProxy("update_planner_lights",
                                                  UpdatePlannerLights)
                res = blinky_proxy([255, 0, 0])  # Set blinky to all red.

                if res.success != 0:
                    rospy.logerr("Blinky request unsuccessful: {}".format(res))

            except Exception as e:
                rospy.logerr("Blinky request unsuccessful: {}".format(e))

            #server.set_aborted()
        print("Initialization complete")
        for i in range(40):
            self.init_thrusters()
            rospy.sleep(0.1)
        print("Thrusters good")
        rospy.sleep(0.5)

    def stop(self):
        """Nothing to do for stop.

        TODO: Stop initialization"""
        pass
