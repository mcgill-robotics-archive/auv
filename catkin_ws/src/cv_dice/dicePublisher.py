#!/usr/bin/env python
import rospy
from darknet_ros_msgs.msg import *
from cv.msg import CvTarget

pub = rospy.Publisher('cv/front_cam_target',CvTarget,queue_size = 1)

def callback(coords):
    xmin = coords.bounding_boxes[0].xmin
    xmax = coords.bounding_boxes[0].xmax
    xtarget = 1.0 * (xmin + xmax) / 2

    ymin = coords.bounding_boxes[0].ymin
    ymax = coords.bounding_boxes[0].ymax
    ytarget = 1.0 * (ymin + ymax) / 2
    
    #TODO: if multiple boxes find the right box!
    probability = coords.bounding_boxes[0].probability

    rospy.loginfo("XTarget: {} YTarget: {}".format(xtarget,ytarget))
    msg = CvTarget()
    msg.gravity.x = xtarget
    msg.gravity.y = ytarget
    #TODO: Add depth    
    msg.gravity.z = 0
    msg.probability.data = probability
    pub.publish(msg)
    


def listener():
    
    rospy.init_node('diceListener',anonymous=True)   
    rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes, callback)
    rospy.spin()





if __name__ == '__main__':
    listener()
