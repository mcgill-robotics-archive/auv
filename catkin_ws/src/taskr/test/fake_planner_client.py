#! /usr/bin/env python
import rospy
from actionlib import SimpleActionClient
from planner.msg import TaskAction, TaskGoal


def fake_planner_client():
    """Fake client that sends a single goal."""
    client = SimpleActionClient("torpedo_task", TaskAction)
    client.wait_for_server()

    # Send testing task once.
    goal = TaskGoal(task="") # ASK Jana what's should be inside. Seems to work without? Is this intentional?

    client.send_goal(goal)

    client.wait_for_result()

    print client.get_result()


if __name__ == '__main__':
    rospy.init_node("fake_planner_client")
    fake_planner_client()
    rospy.spin()
