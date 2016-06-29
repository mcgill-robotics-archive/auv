#! /usr/bin/env python
import sys
import rospy
from actionlib import SimpleActionClient
from planner.msg import TaskAction, TaskGoal


def fake_planner_client(task_name="square"):
    """Fake client that sends a single goal.

    Keyword arguments:
    task_name -- the task to test (default square)
    """
    print(task_name)
    client = SimpleActionClient(task_name + "_task", TaskAction)
    client.wait_for_server()

    # Send testing task once.
    goal = TaskGoal(task=task_name)
    client.send_goal(goal)
    client.wait_for_result()
    print client.get_result()


if __name__ == '__main__':
    rospy.init_node("fake_planner_client")

    # Check for command line arg to specify task.
    if len(sys.argv) == 1:
        fake_planner_client()
    elif len(sys.argv) == 2:
        fake_planner_client(sys.argv[1])
    rospy.spin()
