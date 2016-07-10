#!/usr/bin/env python

import rospy
import yaml
from move import Move
from shoot import Shoot
from initialize import Initializer
from visual_servo import VisualServo
from acoustic_servo import AcousticServo
from rospkg import RosPack
from actionlib import SimpleActionServer
from planner.msg import TaskFeedback, TaskResult, TaskAction

TASK_PATH = RosPack().get_path("taskr") + "/tasks/"


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

    def create_action(self, action, value):
        """Creates the appropriate action object using the name of the action
        from the YAML key.

        Args:
            action: Name of the action.
            value: Args of the YAML, of undefined type. The action is
                   responsible for knowing the type, and the user is
                   responsible for providing the appropriate data in the
                   YAML.
        """
        if action == "move":
            return Move(value)
        elif action == "shoot":
            return Shoot(value)
        elif action == "visual_servo":
            return VisualServo(value)
        elif action == "acoustic_servo":
            return AcousticServo(value)
        elif action == "initialize":
            return Initializer(value)
        else:
            rospy.logerr("{} is not a known type.".format(action))

    def action_sequence(self, data):
        """Iterates through the action sequence defined in the task's YAML file
        and completes each of the actions.

        Args:
            data: Dictionary data from the YAML file.
        """
        for actions in data["actions"]:
            # There should only be one top level key so one iteration of this inner loop.
            for (key, value) in actions.iteritems():
                print key, value
                action = self.create_action(key, value)
                action.start(self._as, self._feedback)
                # Check whether the action has been preempted by the client.
                if self._as.is_preempt_requested():
                    rospy.loginfo("{}: Preempted".format(self._action_name))
                    self._as.set_preempted()
                    self._result.success = False
                    return

        # If an action aborted the server, return.
        if not self._as.is_active():
            return

        self._result.success = True
        self._as.set_succeeded(self._result)


class Initialize(Task):
    YAML = "initialize.yaml"

    def __init__(self, name):
        super(Initialize, self).__init__(name, self.execute_cb)

        # Load the YAML file.
        with open(yaml.load(TASK_PATH + self.YAML)) as f:
            self.data = yaml.load(f)

    def execute_cb(self, goal):
        self.action_sequence(self.data)


class Bins(Task):
    YAML = "bins.yaml"

    def __init__(self, name):
        super(Bins, self).__init__(name, self.execute_cb)

        # Load the YAML file.
        with open(yaml.load(TASK_PATH + self.YAML)) as f:
            self.data = yaml.load(f)

    def execute_cb(self, goal):
        self.action_sequence(self.data)


class Buoys(Task):
    YAML = "buoys.yaml"

    def __init__(self, name):
        super(Buoys, self).__init__(name, self.execute_cb)

        # Load the YAML file.
        with open(yaml.load(TASK_PATH + self.YAML)) as f:
            self.data = yaml.load(f)

    def execute_cb(self, goal):
        self.action_sequence(self.data)


class Gate(Task):
    YAML = "gate.yaml"

    def __init__(self, name):
        super(Gate, self).__init__(name, self.execute_cb)

        # Load the YAML file.
        with open(yaml.load(TASK_PATH + self.YAML)) as f:
            self.data = yaml.load(f)

    def execute_cb(self, goal):
        self.action_sequence(self.data)


class Maneuver(Task):
    YAML = "maneuver.yaml"

    def __init__(self, name):
        super(Maneuver, self).__init__(name, self.execute_cb)

        # Load the YAML file.
        with open(yaml.load(TASK_PATH + self.YAML)) as f:
            self.data = yaml.load(f)

    def execute_cb(self, goal):
        self.action_sequence(self.data)


class Octagon(Task):
    YAML = "octagon.yaml"

    def __init__(self, name):
        super(Octagon, self).__init__(name, self.execute_cb)

        # Load the YAML file.
        with open(yaml.load(TASK_PATH + self.YAML)) as f:
            self.data = yaml.load(f)

    def execute_cb(self, goal):
        self.action_sequence(self.data)


class Torpedo(Task):
    YAML = "torpedo.yaml"

    def __init__(self, name):
        super(Torpedo, self).__init__(name, self.execute_cb)

        # Load the YAML file.
        with open(yaml.load(TASK_PATH + self.YAML)) as f:
            self.data = yaml.load(f)

    def execute_cb(self, goal):
        self.action_sequence(self.data)


class Square(Task):
    """Test task."""
    YAML = "square.yaml"

    def __init__(self, name):
        super(Square, self).__init__(name, self.execute_cb)

        # Load the YAML file.
        with open(yaml.load(TASK_PATH + self.YAML)) as f:
            self.data = yaml.load(f)

    def execute_cb(self, goal):
        self.action_sequence(self.data)


if __name__ == '__main__':
    rospy.init_node("taskr")

    # Initialize tasks.
    Initialize("initialize_task")
    Bins("bin_task")
    Buoys("buoy_task")
    Gate("gate_task")
    Maneuver("maneuver_task")
    Octagon("octagon_task")
    Torpedo("torpedo_task")
    Square("square_task")

    rospy.spin()
