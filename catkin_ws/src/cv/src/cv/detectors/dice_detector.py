#!/usr/bin/env python
import rospy
from darknet_ros_msgs.msg import *
from cv.msg import CvTarget

class DiceDetector():

    def __init__(self,diceName):

        self.pub = rospy.Publisher('cv/front_cam_target',CvTarget,queue_size = 1)
        self.sub = rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes, callback)
        self.diceName = diceName
        self.foundCorrectDie = False

    def callback(self, yolo_msg):
        allBoxes = yolo_msg.bounding_boxes

        #TODO: if two boxes have the same coordinates, ignore the less probable box


        correctDie = None
        #TODO: for now we're taking the first found die with the right number
        for box in allBoxes:
            if (box.Class == self.diceName):
                correctDie = box
                break

        if (correctDie == None):
            self.foundCorrectDie = False

        else:
            xmin = correctDie.xmin
            xmax = correctDie.xmax
            #Xcenter of Die
            xTarget = 1.0 * (xmin + xmax) / 2

            ymin = correctDie.ymin
            ymax = correctDie.ymax
            #Ycenter of Die
            yTarget = 1.0 * (ymin + ymax) / 2

            #TODO: if multiple boxes find the right box!
            probability = correctDie.probability

            #rospy.loginfo("XTarget: {} YTarget: {}".format(xTarget,ytarget))
            msg = CvTarget()
            msg.gravity.x = xTarget
            msg.gravity.y = yTarget
            #TODO: Add depth
            msg.gravity.z = 0
            msg.probability.data = probability
            self.pub.publish(msg)
