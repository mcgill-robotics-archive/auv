#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from darknet_ros_msgs.msg import *
from cv.msg import CvTarget
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


#pub = rospy.Publisher('cv/laneTarget',CvTarget,queue_size = 1)
bridge = CvBridge()

def callback(data):
    try:
        img = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    #This is orange_filter:
    img_hvt = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_orange = np.array([5, 50, 50])
    upper_orange = np.array([30, 255, 255])
    mask = cv2.inRange(img_hvt, lower_orange, upper_orange)
    res = cv2.bitwise_and(img_hvt, img_hvt, mask=mask)

    im2, cnt, hier = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    print(len(cnt))

    if len(cnt) != 0:

        # find contour with biggest area
        biggestCont = max(cnt, key=cv2.contourArea)

        # check contour size
        if (cv2.contourArea(biggestCont) < MIN_CONTOUR_SIZE):
            # TODO: ADD ABORT CONDITIONS
            print("No lane of sufficient size found")

        # draw in blue the contours that were found
        cv2.drawContours(img, [biggestCont], -1, (255, 0, 0), 3, 8)

        # create emptyImage for mask
        contourMask = np.zeros(img.shape, np.uint8)
        cv2.drawContours(contourMask, [biggestCont], -1, (255, 255, 255), cv2.FILLED, 8)

        edges = cv2.Canny(contourMask, 100, 200)

        # TODO: check if these values work reliably
        # Hough Lines
        rho = 1  # distance resolution in pixels of the Hough grid
        theta = np.pi / 180  # angular resolution in radians of the Hough grid
        threshold = 5  # minimum number of votes (intersections in Hough grid cell)
        min_line_length = 100  # minimum number of pixels making up a line
        max_line_gap = 100  # maximum gap in pixels between connectable line segments
        line_image = np.copy(img) * 0  # creating a blank to draw lines on

        # Run Hough on edge detected image
        # Output "lines" is an array containing endpoints of detected line segments
        lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
                                min_line_length, max_line_gap)

        print("Found Lines: {}".format(len(lines)))

        # draw Lines
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(img, (x1, y1), (x2, y2), (100, 230, 230), 5)

        # average all lines with similar direction together
        slopes = []
        for line in lines:
            for x1, y1, x2, y2 in line:
                slope = 1.0 * (y2 - y1) / (x2 - x1)
            slopes.append(slope)

        # stores the line Coords
        lineGroups = [[lines[0]]]
        # stores the slopes of all
        slopeCollection = [[slopes[0]]]

        difSlopes = []
        difSlopes.append(slopes[0])

        for no, slope in enumerate(slopes[1:], start=1):
            similarFound = False
            for idx, pos in enumerate(difSlopes):

                # TODO: possibly adapt values
                lowerBound = pos - (0.2 * abs(pos)) - 0.1
                upperBound = pos + (0.2 * abs(pos)) + 0.1

                # case where slope is similar
                if slope >= lowerBound and slope <= upperBound:
                    slopeCollection[idx].append(slope)
                    lineGroups[idx].append(lines[no])
                    similarFound = True
                    # set the slope of the group to the mean
                    difSlopes[idx] = np.mean(slopeCollection[idx], 0)
                    break
            if similarFound == False:
                difSlopes.append(slope)
                slopeCollection.append([slope])
                lineGroups.append([lines[no]])

        print("Number of Found Slopes: {}".format(len(difSlopes)))

        # find the mean point of lineGroup to calculate intersection
        intercepts = []
        for i, lg in enumerate(lineGroups):
            xPoint = int((np.mean(lg, 0)[0, 0] + np.mean(lg, 0)[0, 2]) / 2)
            yPoint = int((np.mean(lg, 0)[0, 1] + np.mean(lg, 0)[0, 3]) / 2)

            intercept = yPoint - (xPoint * difSlopes[i])
            intercepts.append(intercept)
            cv2.circle(img, (xPoint, yPoint), 10, (0, 255, 0), -1)

        if len(difSlopes) == 2:
            # calculate Intersection Point
            intersectionX = int((intercepts[1] - intercepts[0]) / (difSlopes[0] - difSlopes[1]))
            intersectionY = int(
                (difSlopes[0] * intercepts[1] - difSlopes[1] * intercepts[0]) / (difSlopes[0] - difSlopes[1]))

            cv2.circle(img, (intersectionX, intersectionY), 30, (0, 255, 0), -1)

            # plt.imshow(line_image)
            # plt.show()

        else:
            print("ERROR: NOT 2 LINEGROUPS")

        rect = cv2.minAreaRect(biggestCont)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(img, [box], 0, (0, 0, 255), 2)

    # finding angle
    angle = rect[2]

    rectSize = rect[1]
    if rectSize[0] > rectSize[1]:
        angle = angle + 90
    angle = -angle
    print("Change Angle by: {} deg".format(angle))

    # TODO make sure the orange area spotted is big enough to avoid turning when only the edge is seen.


    cv2.imshow('image', img)
    cv2.waitKey(3000)
    


def listener():
    
    rospy.init_node('laneChecker',anonymous=True)
    rospy.Subscriber("/camera_front/image_rect",Image, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
