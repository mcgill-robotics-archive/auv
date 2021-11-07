import roslib
roslib.load_manifest('cv')
import rospy
import actionlib

from cv.msg import Call_ORBAction ,Call_ORBGoal

if __name__ == '__main__':
    rospy.init_node('ORB_client')
    print('ORB Client Started')
    client = actionlib.SimpleActionClient('ORB_detector', Call_ORBAction )
    print('ORB Waiting')
    client.wait_for_server()

    goal = Call_ORBGoal()
    # Fill in the goal here
    client.send_goal(goal)
    print('ORB Sent')
    client.wait_for_result(rospy.Duration.from_sec(5.0))
    print('ORB Complete')