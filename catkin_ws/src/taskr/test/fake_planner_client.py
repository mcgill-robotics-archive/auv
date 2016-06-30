#! /usr/bin/env python
import sys
import rospy
from actionlib import SimpleActionClient
from planner.msg import TaskAction, TaskGoal

# The list of the possible tasks to test
TASKS = ["bin", "buoy", "initialize", "gate", "maneuver", "octagon", "square",
         "torpedo"]

class InvalidTaskException(Exception):
    """Custom error task for command line argument check."""
    pass


def fake_planner_client(task_name="square"):
    """Fake client that sends a single goal.

    Keyword arguments:
    task_name -- the task to test (default square)
    """
    client = SimpleActionClient(task_name + "_task", TaskAction)
    client.wait_for_server()

    # Send testing task once.
    goal = TaskGoal(task=task_name)
    client.send_goal(goal)
    client.wait_for_result()
    print client.get_result()


def validate_and_execute(task_name):
    """Make sure that the command line arg is a valid task before executing."""
    if task_name in TASKS:
        fake_planner_client(sys.argv[1])
    else:
        e_msg = ("Task is not valid.\n Check for typo or add to TASK global.")
        raise InvalidTaskException(e_msg)


if __name__ == '__main__':
    rospy.init_node("fake_planner_client")

    # Check for command line arg to specify task.
    if len(sys.argv) == 1:
        fake_planner_client()
    elif len(sys.argv) == 2:
        validate_and_execute(sys.argv[1])

    rospy.spin()
