#!/usr/bin/env python

import rospy
import numpy as np
from move import Move
from shoot import Shoot
from turn import Turn
from dive import Dive
from initialize import Initializer
from acoustic_servo import AcousticServo
from actionlib import SimpleActionServer
from auv_msgs.msg import TaskStatus
from std_msgs.msg import Float64
from planner.msg import TaskFeedback, TaskResult, TaskAction
from auv_msgs.msg import HydrophonesAction, HydrophonesFeedback, HydrophonesResult

current_task = TaskStatus()
current_task.task = TaskStatus.TASK_IDLE
current_task.action = TaskStatus.ACTION_IDLE

# Maps to the objects and states.
action_object_map = {"move": Move,
                     "turn": Turn,
                     "dive": Dive,
                     "shoot": Shoot,
                     "initialize": Initializer,
                     "acoustic_servo": AcousticServo}

action_state_map = {"move": TaskStatus.MOVE,
                    "shoot": TaskStatus.SHOOT,
                    "initialize": TaskStatus.INITIALIZE,
                    "acoustic_servo": TaskStatus.OCTAGON,
                    "turn": TaskStatus.MOVE,
                    "dive": TaskStatus.MOVE}


class Task(object):

    """Task object.

    Base class for each task, responsible for executing the actions belonging
    to each task as defined in a configuration file.
    """

    _feedback = TaskFeedback()
    _result = TaskResult()

    def __init__(self, name, function):
        # Create move action server
        self._action_name = name
        self._as = SimpleActionServer(
            self._action_name, TaskAction,
            execute_cb=function,
            auto_start=False
        )
        self._as.start()

        # Keep a list of running action objects here, so that they can be
        # stopped easily if preempted.
        self.running_actions = []

        self.preempted = False
        self.preempt_timer = rospy.Timer(rospy.Duration(0.5), self.preempt_check)

    def action_sequence(self, data):
        """Iterates through the action sequence defined in the task's YAML file
        and completes each of the actions.

        Args:
            data: Dictionary data from the YAML file.
        """
        for actions in data["actions"]:
            # There should only be one top level key so one iteration of this inner loop.
            for (key, value) in actions.iteritems():
                if key in action_object_map and key in action_state_map:
                    # Get the correct object and state from the dictionaries.
                    current_task.action = action_state_map[key]
                    action = action_object_map[key](value)

                    # Start action.
                    self.running_actions.append(action)
                    action.start(self._as, self._feedback)

                    # Remove the action when done.
                    if action in self.running_actions:
                        action.stop()
                        self.running_actions.remove(action)
                else:
                    rospy.logerr("{} is not a known type.".format(key))

                if self.preempted:
                    break

        self.stop_all()

        current_task.task = TaskStatus.TASK_IDLE
        current_task.action = TaskStatus.ACTION_IDLE

        # If all is still well with the action (not failed or preempted), set
        # successful. This isn't very intelligent right now.
        if self._as.is_active():
            self._result.success = True
            self._as.set_succeeded(self._result)

    def preempt_check(self, _):
        # Check whether the action has been preempted by the client.
        if not self._as.is_active():
            return

        if self._as.is_preempt_requested() or rospy.is_shutdown():
            rospy.loginfo("{}: Preempted".format(self._action_name))
            self._as.set_preempted()
            self._result.success = False
            self.stop_all()
            self.preempted = True

    def stop_all(self):
        for action in self.running_actions:
            action.stop()
            self.running_actions.remove(action)


class Initialize(Task):
    YAML = "initialize.yaml"

    def __init__(self, name):
        super(Initialize, self).__init__(name, self.execute_cb)

        self.data = rospy.get_param("taskr/initialize")

    def execute_cb(self, goal):
        current_task.task = TaskStatus.INITIALIZE
        self.action_sequence(self.data)


class Bins(Task):
    YAML = "bins.yaml"

    def __init__(self, name):
        super(Bins, self).__init__(name, self.execute_cb)

        self.data = rospy.get_param("taskr/bins")

    def execute_cb(self, goal):
        current_task.task = TaskStatus.BINS
        self.action_sequence(self.data)


class Buoys(Task):
    YAML = "buoy.yaml"

    def __init__(self, name):
        super(Buoys, self).__init__(name, self.execute_cb)

        self.data = rospy.get_param("taskr/buoy")

    def execute_cb(self, goal):
        current_task.task = TaskStatus.BUOYS
        self.action_sequence(self.data)


class Gate(Task):
    YAML = "gate.yaml"

    def __init__(self, name):
        super(Gate, self).__init__(name, self.execute_cb)

        self.data = rospy.get_param("taskr/gate")

    def execute_cb(self, goal):
        current_task.task = TaskStatus.GATE
        self.action_sequence(self.data)


class Maneuver(Task):
    def __init__(self, name):
        super(Maneuver, self).__init__(name, self.execute_cb)

        self.data = rospy.get_param("taskr/maneuver")

    def execute_cb(self, goal):
        current_task.task = TaskStatus.MANEUVER
        self.action_sequence(self.data)


class Octagon(Task):
    YAML = "octagon.yaml"

    def __init__(self, name):
        super(Octagon, self).__init__(name, self.execute_cb)

        self.data = rospy.get_param("taskr/octagon")

    def execute_cb(self, goal):
        current_task.task = TaskStatus.OCTAGON
        self.action_sequence(self.data)


class Torpedo(Task):
    YAML = "torpedo.yaml"

    def __init__(self, name):
        super(Torpedo, self).__init__(name, self.execute_cb)

        self.data = rospy.get_param("taskr/torpedo")

    def execute_cb(self, goal):
        current_task.task = TaskStatus.TORPEDO
        self.action_sequence(self.data)


class Square(Task):
    """Test task."""
    YAML = "square.yaml"

    def __init__(self, name):
        super(Square, self).__init__(name, self.execute_cb)

        self.data = rospy.get_param("taskr/square")

    def execute_cb(self, goal):
        current_task.task = TaskStatus.SQUARE
        self.action_sequence(self.data)


class Wait(object):

    SLEEP_TIME = rospy.get_param("taskr/wait_time", default=20)
    MOVE_RATE = rospy.get_param("taskr/wait_move_rate", default=1)

    def __init__(self):
        self._action_name = "wait"
        self._as = SimpleActionServer(
            self._action_name, TaskAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._as.start()

    def execute_cb(self, goal):
        start_time = rospy.Time.now()
        rospy.loginfo("Sleeping for {} secs".format(self.SLEEP_TIME))

        rate = rospy.Rate(self.MOVE_RATE)
        feedback = TaskFeedback()

        while (rospy.Time.now() - start_time) < rospy.Duration(self.SLEEP_TIME):
            move_cmd = {"distance": 0}
            move_action = Move(move_cmd)
            move_action.start(self._as, feedback)

            if self._as.is_preempt_requested():
                rospy.logerr("Wait preempted")
                self._as.set_preempted()
                return

            rate.sleep()
            move_action.stop()

        rospy.loginfo("Done sleeping")

        result = TaskResult()
        result.success = True
        self._as.set_succeeded(result)


class ChooseTask(object):
    """Choose which task to do based on hydrophones."""

    WINDOW_GOAL = 1

    def __init__(self):
        self._action_name = "hydro_choose_task"
        self._as = SimpleActionServer(
            self._action_name, HydrophonesAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.goals = []

        self.hydro_sub = rospy.Subscriber("hydrophones/heading", Float64, self.hydro_cb)

        self._as.start()

    def execute_cb(self, goal):
        rate = rospy.Rate(2)
        rospy.loginfo("Collecting data!")
        while len(self.goals) < self.WINDOW_GOAL:
            if self._as.is_preempt_requested():
                rospy.logerr("Hydrophones preempted")
                self._as.set_preempted()
                return

            feedback = HydrophonesFeedback()
            feedback.hydro_heading = np.mean(self.goals) if self.goals else 0
            self._as.publish_feedback(feedback)
            move_cmd = {"distance": 0.5}
            move_action = Move(move_cmd)
            move_action.start(self._as, feedback)

            rate.sleep()
            move_action.stop()

        rospy.loginfo("Enough data has been collected.")

        hydro_heading = np.mean(self.goals)

        rospy.loginfo("Got heading {}".format(hydro_heading))

        result = HydrophonesResult()

        if hydro_heading > 0:  # Positive angle
            rospy.loginfo("Hydrophones are to the right.")
            result.quadrant = "right"
        else:
            rospy.loginfo("Hydrophones are to the left.")
            result.quadrant = "left"

        self._as.set_succeeded(result)

    def hydro_cb(self, msg):
        self.goals.append(msg.data)


def publish_task(event):
    """Callback for timer which publishes the current task being attempted."""
    task_pub.publish(current_task)


if __name__ == '__main__':
    rospy.init_node("taskr")

    task_pub = rospy.Publisher("task", TaskStatus, queue_size=1)
    rospy.Timer(rospy.Duration(0.2), publish_task)

    # For debugging purposes, to ensure the file is correct.
    TASK_PATH = rospy.get_param("taskr/task_file")
    rospy.loginfo("YAML path: {}".format(TASK_PATH))

    # Initialize tasks.
    Initialize("initialize_task")
    Bins("bin_task")
    Buoys("buoy_task")
    Gate("gate_task")
    Maneuver("maneuver_task")
    Octagon("octagon_task")
    Torpedo("torpedo_task")
    Square("square_task")

    Wait()
    ChooseTask()

    rospy.spin()
