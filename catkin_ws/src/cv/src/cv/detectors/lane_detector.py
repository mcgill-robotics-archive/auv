#!/usr/bin/env python
import rospy
import cv2
import math
import numpy as np
from cv.msg import CvTarget
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class LaneDetector():

    def __init__(self):
        self.pub = rospy.Publisher('cv/down_cam_target', CvTarget, queue_size=1)
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/camera_down/image_raw", Image, self.callback)

        self.angle_top_lane = None
        self.laneFound = False

    def getAngleOfTopLine(self, points, difSlopes, img):
        # find the line with higher YCoordinate
        maxY = np.inf
        maxIdx = -1
        for idx, (x, y) in enumerate(points):
            if y < maxY:
                maxIdx = idx
                maxY = y

        # debug
        cv2.circle(img, points[maxIdx], 25, (0, 255, 255), -1)
        angle = math.degrees(math.atan(difSlopes[maxIdx]))
        # angle = math.atan(difSlopes[maxIdx])
        if angle >= 0:
            angle = 90 - angle
        if angle < 0:
            angle = -(90 + angle)

        self.angle_top_lane = math.radians(angle)

        #print("Turn by {} degrees!".format(angle))


    def callback(self,data):
        try:
            #rospy.loginfo("retrieving image")
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # This is orange_filter:
        MIN_CONTOUR_SIZE = rospy.get_param("/cv/lanes/orange_cnt_size", 10000)
        img_hvt = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_orange = np.array([5, 50, 50])
        upper_orange = np.array([30, 255, 255])
        mask = cv2.inRange(img_hvt, lower_orange, upper_orange)

        im2, cnt, hier = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        if len(cnt) == 0:
            #print("No lane of sufficient size found")
            self.laneFound = False
            self.angle_top_lane = None
            return

        else:
            #Contours were found
            # find contour with biggest area
            biggestCont = max(cnt, key=cv2.contourArea)
            # check contour size

            if (cv2.contourArea(biggestCont) < MIN_CONTOUR_SIZE):
                #print("No lane of sufficient size found")
                self.laneFound = False
                self.angle_top_lane = None
                return
            else:
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
                threshold = 20  # minimum number of votes (intersections in Hough grid cell)
                min_line_length = 40  # minimum number of pixels making up a line
                max_line_gap = 100  # maximum gap in pixels between connectable line segments
                line_image = np.copy(img) * 0  # creating a blank to draw lines on
                # Run Hough on edge detected image
                # Output "lines" is an array containing endpoints of detected line segments
                lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
                                        min_line_length, max_line_gap)
                #print("Found Lines: {}".format(len(lines)))

                # draw Lines
                for line in lines:
                    for x1, y1, x2, y2 in line:
                        cv2.line(img, (x1, y1), (x2, y2), (100, 230, 230), 5)

                # average all lines with similar direction together
                slopes = []
                for line in lines:
                    for x1, y1, x2, y2 in line:
                        slope = 1.0 * (y2 - y1) / (x2 - x1)
                        # to avoid infinite slope
                        if slope == float("Inf") or slope > 25:
                            slope = 25
                        if slope == -float("Inf") or slope < -25:
                            slope = 25

                    slopes.append(slope)

                # stores the line Coords
                lineGroups = [[lines[0]]]
                # stores the slopes of all
                slopeCollection = [[slopes[0]]]
                difSlopes = []
                difSlopes.append(slopes[0])

                #group lines with similar slope together
                for no, slope in enumerate(slopes[1:], start=1):
                    similarFound = False
                    for idx, pos in enumerate(difSlopes):
                        # TODO: possibly adapt values
                        lowerBound = pos - (0.4 * abs(pos)) - 0.1
                        upperBound = pos + (0.4 * abs(pos)) + 0.1

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

                #print("Number of Found Slopes: {}".format(len(difSlopes)))

                # find the mean point of lineGroup to calculate intersection
                # sort LineGroup by size to get the groups with most entries
                relevantLines = zip(lineGroups, difSlopes)
                relevantLines.sort(key=lambda x: len(x[0]), reverse=True)
                lineGroups, difSlopes = zip(*relevantLines)

                #debug
                #for l in lineGroups:
                    #print(len(l))
                #print(difSlopes)

                xReturn = None
                yReturn = None

                centerPoints = []
                intercepts = []


                if (len(lineGroups) == 1):
                    # there is only one slope found
                    lg = lineGroups[0]
                    #print("NOT 2 LINEGROUPS, calculating center")
                    xPoint = int((np.mean(lg, 0)[0, 0] + np.mean(lg, 0)[0, 2]) / 2)
                    yPoint = int((np.mean(lg, 0)[0, 1] + np.mean(lg, 0)[0, 3]) / 2)
                    cv2.circle(img, (xPoint, yPoint), 30, (255, 0, 0), -1)
                    xReturn = xPoint
                    yReturn = yPoint


                elif(len(lineGroups) >= 2):
                    # only look at the 2 biggest groups (ignores irrelevant lines)
                    for i, lg in enumerate(lineGroups[:2]):
                        xPoint = int((np.mean(lg, 0)[0, 0] + np.mean(lg, 0)[0, 2]) / 2)
                        yPoint = int((np.mean(lg, 0)[0, 1] + np.mean(lg, 0)[0, 3]) / 2)

                        intercept = yPoint - (xPoint * difSlopes[i])
                        intercepts.append(intercept)
                        #debug
                        cv2.circle(img, (xPoint, yPoint), 10, (0, 255, 0), -1)
                        centerPoints.append((xPoint, yPoint))

                    #only look at the 2 most occuring slopes
                    difSlopes = difSlopes[:2]

                    # calculate Intersection Point
                    intersectionX = int((intercepts[1] - intercepts[0]) / (difSlopes[0] - difSlopes[1]))
                    intersectionY = int(
                        (difSlopes[0] * intercepts[1] - difSlopes[1] * intercepts[0]) / (difSlopes[0] - difSlopes[1]))

                    #calculate the angle between lanes to make sure a correct lane was found
                    angle = math.degrees(math.atan((difSlopes[0] - difSlopes[1]) / (1 + difSlopes[0] * difSlopes[1])))
                    #print("Angle = {}".format(angle))

                    if abs(angle) > 30:
                        # this is the normal case of 2 lane parts found
                        cv2.circle(img, (intersectionX, intersectionY), 30, (0, 255, 0), -1)
                        xReturn = intersectionX
                        yReturn = intersectionY

                        self.getAngleOfTopLine(centerPoints, difSlopes, img)

                    else:
                        # 2 different slopes found that don't have big angle
                        # finds 2 slopes even tho it should be just one
                        xMean = 0
                        yMean = 0
                        for lg in lineGroups[:2]:
                            xMean += (np.mean(lg, 0)[0, 0] + np.mean(lg, 0)[0, 2]) / 2
                            yMean += (np.mean(lg, 0)[0, 1] + np.mean(lg, 0)[0, 3]) / 2
                        xMean = int(xMean / 2.0)
                        yMean = int(yMean / 2.0)

                        cv2.circle(img, (xMean, yMean), 30, (255, 0, 255), -1)
                        xReturn = xMean
                        yReturn = yMean

                small = cv2.resize(img, (0, 0), fx=0.5, fy=0.5)
                cv2.imshow("image", small)
                cv2.waitKey(5)

                if (xReturn != None):
                    msg = CvTarget()
                    msg.gravity.x = xReturn
                    msg.gravity.y = yReturn
                    msg.gravity.z = 0
                    msg.probability.data = 1.0
                    self.pub.publish(msg)
                    self.laneFound = True

    def getAngle(self):
        return self.angle_top_lane

    def getLaneFound(self):
        return self.laneFound

    def stop(self):
        self.sub.unregister()

