import roslib
roslib.load_manifest('cv')
import rospy
import actionlib
import cv2
from cv_bridge import CvBridge, CvBridgeError

from cv.msg import Call_ORBAction ,Call_ORBGoal

if __name__ == '__main__':

    #Load up the target image
    #Hardcoded for now. TODO : fix hardcoding!
    img_target = cv2.imread('DarkTower.png',0)
    
    #cv2.imshow('target' , img_target)
    #cv2.waitKey(0)
    
    bridge = CvBridge()
    try:
        #img = bridge.cv2_to_imgmsg(img_target, encoding="passthrough")
        img = bridge.cv2_to_imgmsg(img_target,"8UC1")
    except CvBridgeError as e:
        print(e)    

    rospy.init_node('ORB_client')
    print('ORB Client Started')
    client = actionlib.SimpleActionClient('ORB_detector', Call_ORBAction )
    print('ORB Waiting')
    client.wait_for_server()

    goal = Call_ORBGoal(TargetImage = img)
    # Fill in the goal here
    client.send_goal(goal)
    print('ORB Sent')
    client.wait_for_result(rospy.Duration.from_sec(5.0))
    print('ORB Complete')