#!/usr/bin/env python

import roslib
import rospy

from sensor_msgs.msg import Image
from bbauv_msgs.srv import sonar_point

from utils.utils import Utils
from front_commons.frontCommsVision import FrontCommsVision as vision

from dynamic_reconfigure.server import Server as DynServer
from utils.config import sonarConfig as Config

import math
import numpy as np
import cv2

class Sonar():
    self.threshold = 160
    self.sobelKern = (5, 11)
    self.lenLowerBound = 45
    self.lenUpperBound = 100

    def __init__(self):
        self.sonarDist = 0.0
        self.sonarBearing = 0.0
        self.registerSonar()

        self.dynServer = DynServer(Config, self.reconfigure)

        self.sonarSrv = rospy.ServiceProxy('sonar_point', sonar_point, persistent=True)

    def gotSonarFrame(self, img):
        img = cv2.resize(img, (vision.screen['width'], vision.screen['height']))

        binImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mask = cv2.threshold(binImg, self.threshold, 255, cv2.THRESH_BINARY)[1]

        scratchImgCol = img
        cv2.putText(scratchImgCol, "SONAR PROCESSED", (20,460),  cv2.FONT_HERSHEY_DUPLEX, 1, (211,0,148))

        zerosmask = np.zeros((480,640,3), dtype=np.uint8)
        sobel = cv2.Sobel(mask, cv2.CV_8U, 0, 1, self.sobelKern)
        dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        # sobel = cv2.dilate(sobel, dilateEl, iterations=2)
        # return cv2.cvtColor(sobel, cv2.COLOR_GRAY2BGR)

        # scratchImgCol = cv2.cvtColor(sobel, cv2.COLOR_GRAY2BGR)
        # cv2.putText(scratchImgCol, "SONAR PROCESSED", (20,460),  cv2.FONT_HERSHEY_DUPLEX, 1, (211,0,148))

        contours, hierarchy = cv2.findContours(sobel, cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)

        allLinesList = []
        allBearingList = []

        for i in contours:
            mask = np.zeros((vision.screen['width'], vision.screen['height']), 
                dtype=np.uint8)
            cv2.drawContours(mask, [i], 0, 255, -1)
            lines = cv2.HoughLinesP(mask, 1, math.pi/2, 1, None, 1, 0)

            if lines is not None:
                line = lines[0]

                pt1List = [(i[0], i[1]) for i in line]
                pt2List = [(i[2],i[3]) for i in line]
                sorted(pt1List, key=lambda x:x[0])
                sorted(pt2List, key=lambda x:x[0])
                pt1 = pt1List[0] if pt1List[0] < pt2List[0] else pt2List[0]
                pt2 = pt1List[-1] if pt1List[-1] > pt2List[-1] else pt2List[-1]

                length = Utils.distBetweenPoints(pt1, pt2)

                if self.lenLowerBound < length < self.lenUpperBound:
                    centerPoint = ((pt1[0]+pt2[0])/2, (pt2[1]+pt2[1])/2 )
                    angle = math.atan2((pt2[1]-pt1[1]), (pt2[0]-pt1[0]))
                    if -30 < angle < 30:

                        allLinesList.append((pt1, pt2))

                        cv2.line(scratchImgCol, pt1, pt2, (0,0,255), 3)
                        angleStr = "{0:.2f}".format(angle)
                        cv2.putText(scratchImgCol, "Ang " + str(angleStr),
                            (int(pt1[0]), int(pt1[1]-5)), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 0), 1)

                        offset = 60
                        point = (vision.screen['height']-offset)
                        dist = ((point-pt1[1])*1.0/point) * 10.0  # 10m the FOV of sonar
                        distStr = "{0:.2f}".format(dist)
                        cv2.putText(scratchImgCol, "Dist " + str(distStr),
                            (int(pt1[0]), int(pt1[1]-20)), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,0), 1)

                        allBearingList.append((angle, dist, centerPoint))

        if len(allBearingList) > 0:
            sorted(allBearingList, key=lambda p:p[1])
            self.sonarDist = allBearingList[0][1]
            self.sonarBearing = allBearingList[0][0]
            point = (int(allBearingList[0][2][0]), int(allBearingList[0][2][1]))

            cv2.circle(scratchImgCol, point, 5, (20, 255, 255), 2)

            # Send the center point over to sonar to get range and bearing
            # self.getSonarPoint(point)

        cv2.putText(scratchImgCol, "Sonar Dist " + str(self.sonarDist),
            (30, 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)

        cv2.putText(scratchImgCol, "Sonar Bearing " + str(self.sonarBearing), (30, 60),
                cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)

        return scratchImgCol

    def getSonarPoint(self, point){
        resp = self.sonarSrv(point[0], point[1])
        self.sonarBearing = resp.bearing
        self.sonarDist = resp.range
        rospy.loginfo("Bearing {} Range {}".format(self.sonarBearing, self.sonarDist))
    }


    def registerSonar(self):
        rospy.loginfo("SONAR SONAR")
        self.sonarSub = rospy.Subscriber("/sonar_image", Image, self.sonarImageCallback)
        self.sonarPub = rospy.Publisher("/sonar_pub", Image)
        rospy.sleep(rospy.Duration(0.05))
    
    def sonarImageCallback(self, rosImg):
        outImg = self.gotSonarFrame(Utils.rosimg2cv(rosImg))
        if outImg is not None:
            try:
                self.sonarPub.publish(Utils.cv2rosimg(outImg))
            except Exception, e:
                pass

        # rospy.sleep(rospy.Duration(0.05))

    def self.reconfigure(self, config, level){
        self.threshold = config.binThres
        self.sobelKern = (config.sobel1, config.sobel2)
        self.lenLowerBound = config.length1
        self.lenUpperBound = config.length2
    }

def main():
    rospy.init_node('SonarRanger')
    sonar = Sonar()
    rospy.spin()
