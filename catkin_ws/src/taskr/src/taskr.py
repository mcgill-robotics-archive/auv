#!/usr/bin/env python

import rospy
import yaml
from move import Move
from shoot import Shoot
from visual_servo import VisualServo
from acoustic_servo import AcousticServo
from rospkg import RosPack
from actionlib import SimpleActionServer
from actionlib import SimpleActionClient
from planner.msg import moveFeedback, moveResult, moveAction
from auv_msgs.msg import SetVelocityAction, SetVelocityGoal

TASK_PATH = RosPack().get_path("taskr") + "/tasks/"


class Taskr(object):
    _feedback = moveFeedback()
    _result = moveResult()

    def __init__(self, name):
        # Create move action server
        self._action_name = name
        self._as = SimpleActionServer(
            self._action_name, moveAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._as.start()

    def createAction(self, action, value):
        if action == "move":
            Move(value)
        elif action == "shoot":
            Shoot(value)
        elif action == "visual_servo":
            VisualServo(value)
        elif action == "acoustic_servo":
            AcousticServo(value)
        else:
            rospy.logerr("{} is not a known type.".format(action))


class Bins(Taskr):
    YAML = "bins.yaml"

    def __init__(self, name):
        self._action_name = name
        self.data = yaml.load(TASK_PATH + self.YAML)

    def execute_cb(self, goal):
        for (key, value) in self.data.iteritems():
            self.createAction(key, value)


class Buoys(Taskr):
    YAML = "buoys.yaml"

    def __init__(self, name):
        self._action_name = name
        self.data = yaml.load(TASK_PATH + self.YAML)

    def execute_cb(self, goal):
        pass


class Gate(Taskr):
    YAML = "gate.yaml"

    def __init__(self, name):
        self._action_name = name
        self.data = yaml.load(TASK_PATH + self.YAML)

    def execute_cb(self, goal):
        pass


class Maneuver(Taskr):
    YAML = "maneuver.yaml"

    def __init__(self, name):
        self._action_name = name
        self.data = yaml.load(TASK_PATH + self.YAML)

    def execute_cb(self, goal):
        pass


class Octagon(Taskr):
    YAML = "octagon.yaml"

    def __init__(self, name):
        self._action_name = name
        self.data = yaml.load(TASK_PATH + self.YAML)

    def execute_cb(self, goal):
        pass


class Torpedo(Taskr):
    YAML = "torpedo.yaml"

    def __init__(self, name):
        self._action_name = name
        self.data = yaml.load(TASK_PATH + self.YAML)

    def execute_cb(self, goal):
        pass


class Square(Taskr):
    """Test task."""
    YAML = "square.yaml"

    _feedback = moveFeedback()
    _result = moveResult()

    def __init__(self, name):
        # Create move action server
        self.data = yaml.load(TASK_PATH + self.YAML)
        self._action_name = name
        self._as = SimpleActionServer(
            self._action_name, moveAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._as.start()

    def execute_cb(self, goal):
        print 'Received goal'
        print goal
        # Create velocity action client
        vel_client = SimpleActionClient('controls_velocity', SetVelocityAction)

        time = int(goal.time.secs)
        velocity = goal.velocity
        theta = goal.theta
        depth = goal.depth

        ctrl_goal = SetVelocityGoal()
        ctrl_goal.cmd.depth = depth

        if theta != 0:
            print 'Sending Yaw'
            # Turn first before surging
            ctrl_goal.cmd.yaw = theta
            # Send to velocity server
            vel_client.send_goal(ctrl_goal)
            # Check if we received preempt request from Planner
            if self._as.is_preempt_requested():
                print "Taskr preempted"
                # Send preempt request to Controls
                vel_client.cancel_goal()
                # Handle preemption
                self._as.set_preempted()
                return
            vel_client.wait_for_result()

        # Surge
        rate = rospy.Rate(10)
        for s in range(1, time * 10 + 1, 1):
            print 'Sending Surge'
            ctrl_goal.cmd.surgeSpeed = velocity.linear.x
            ctrl_goal.cmd.yaw = theta
            vel_client.send_goal(ctrl_goal)
            # Check if we received preempt request from Planner
            if self._as.is_preempt_requested():
                print "Taskr preempted"
                # Send preempt request to Controls
                vel_client.cancel_goal()
                # Handle preemption
                self._as.set_preempted()
                return
            # Sleep for the amount of time that makes the for-loop run
            # for 1 second.
            rate.sleep()

        self._result.success = True
        self._as.set_succeeded(self._result)
        print 'Success!'


if __name__ == '__main__':
    rospy.init_node('taskr')
    # Taskr('square_action')

    # Initialize tasks.
    Bins("bin_task")
    Buoys("bouy_task")
    Gate("gate_task")
    Maneuver("maneuver_task")
    Octagon("octagon_task")
    Torpedo("torpedo_task")
    Square("square_task")

    rospy.spin()
