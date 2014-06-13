#/usr/bin/env/python 

'''
Torpedo Vision
'''

import math
import numpy as np
import cv2

import rospy

from utils.utils import Utils
from front_commons.frontCommsVision import FrontCommsVision as vision

class TorpedoVision:    
    # Vision parameters
    
    thresParams = {
                    'lo': (122, 0, 0), 'hi': (170, 100, 255), 
#                     'lo': (95, 100, 0), 'hi': (166, 255, 255), 
                   'dilate': (5,5), 'erode': (3,3), 'open': (3,3)}

    greenParams = {
                   'lo': (46, 100, 0), 'hi': (85, 255, 255),
                   'dilate': (5,5), 'erode': (3,3), 'open': (3,3)
                  }
    
    circleParams = {'minRadius': 5, 'maxRadius': 100}
    cannyParams = {'loThres': 100, 'hiThres': 150}    
    
    minContourArea = 100
    
    previousCentroid = (-1, -1)
    previousRadius = 0
        
    def __init__(self, comms = None, debugMode = True):
        self.debugMode = debugMode
        self.comms = comms
        
    def gotFrame (self, img):
        # Set up parameters
        outImg = None
        
        allCentroidList = []
        allRadiusList = []
        
        # Preprocessing 
        rawImg = vision.preprocessImg(img)
        rawImg = vision.shadesOfGray(rawImg)

        blurImg = cv2.GaussianBlur(rawImg, ksize=(0, 0), sigmaX=10)
        enhancedImg = cv2.addWeighted(rawImg, 2.5, blurImg, -1.5, 0)

        hsvImg = cv2.cvtColor(rawImg, cv2.COLOR_BGR2HSV)
        hsvImg = self.normaliseImg(hsvImg)
        # return cv2.cvtColor(hsvImg, cv2.COLOR_HSV2BGR)

        # Threshold out something
        # binImg = cv2.inRange(hsvImg, self.thresParams['lo'], self.thresParams['hi'])
        binImg = cv2.inRange(hsvImg, self.greenParams['lo'], self.greenParams['hi'])
        binImg = vision.erodeAndDilateImg(binImg, self.greenParams)
        # return cv2.cvtColor(binImg, cv2.COLOR_GRAY2BGR)
 
        # dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))        
        # binImg = cv2.dilate(binImg, dilateEl)
        if binImg is not None:
            self.comms.foundSomething = True

        # return cv2.cvtColor(binImg, cv2.COLOR_GRAY2BGR)
        
#         if edges is not None:
#             self.comms.foundSomething = True 
        
        # Find Hough circles - used to be edges
        scratchImgCol = cv2.cvtColor(binImg, cv2.COLOR_GRAY2BGR)
        scratchImg = binImg.copy()
        circles = cv2.HoughCircles(scratchImg, cv2.cv.CV_HOUGH_GRADIENT, 1,
                                   minDist=10, param1=78, param2=16,
                                   minRadius = 8,
                                   maxRadius = self.circleParams['maxRadius'])        
        if circles is None:
            self.comms.foundCount = self.comms.foundCount + 1
            return scratchImgCol
        
        self.comms.foundCircles = True
        circlesSorted = np.array(sorted(circles[0], key=lambda x:x[2], reverse=True))
        
        for circle in circlesSorted:
            circleCentroid = (circle[0], circle[1])
            allCentroidList.append(circleCentroid)
            allRadiusList.append(circle[2])
            # Draw Circles
            cv2.circle(scratchImgCol, circleCentroid, circle[2], (255, 255, 0), 2)
            cv2.circle(scratchImgCol, circleCentroid, 2, (255, 0, 255), 3)

        
        # Centroid resetted
        if self.comms.centroidToShoot is None:
            # Pick the largest circle
            if self.comms.numShoot == 0 or len(circles) < 2:
                self.comms.centroidToShoot = (circlesSorted[0][0], circlesSorted[0][1])
                self.comms.radius = circlesSorted[0][2]
            elif self.comms.numShoot == 1:
                self.comms.centroidToShoot = (circlesSorted[1][0], circlesSorted[1][1])
                self.comms.radius = circlesSorted[1][2]
            rospy.loginfo(self.comms.centroidToShoot)
        else:
            # Find the centroid closest to the previous 
            if len(allCentroidList) != 0:
                for centroid in allCentroidList:
                    distDiff = []
                    distDiff.append(Utils.distBetweenPoints(
                                        self.previousCentroid, centroid))
                    minIndex = distDiff.index(min(distDiff))
                    self.comms.centroidToShoot = allCentroidList[minIndex]
                    self.comms.radius = allRadiusList[minIndex]
#                     self.comms.medianCentroid.append(allCentroidList[minIndex])
#                     self.comms.medianRadius.append(allRadiusList[minIndex])          
            else:
                # If not then just use the previous one 
                self.comms.centroidToShoot = self.previousCentroid
                self.comms.radius = self.previousRadius
                               
        # Draw centroid to shoot 
        # Find median of (x,y)
#         x = [a[0] for a in self.comms.medianCentroid]
#         y = [a[1] for a in self.comms.medianCentroid]
#         self.comms.centroidToShoot = (int(np.median(x)), int(np.median(y)))
#         self.comms.radius = np.median(self.comms.medianRadius)
        self.previousCentroid = self.comms.centroidToShoot   
        self.previousRadius = self.comms.radius
        cv2.circle(scratchImgCol, self.comms.centroidToShoot, 3, (0, 255, 255), 2)
        rospy.loginfo("Radius: {}".format(self.comms.radius))
            
        # How far the centroid is off the screen center
        self.comms.deltaX = float((self.comms.centroidToShoot[0] - vision.screen['width']/2)*1.0/
                                    vision.screen['width'])

        cv2.putText(scratchImgCol, "X  " + str(self.comms.deltaX), (30,30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255))
        self.comms.deltaY = float((self.comms.centroidToShoot[1] - vision.screen['height']/2)*1.0/
                                  vision.screen['height'])
        cv2.putText(scratchImgCol, "Y  " + str(self.comms.deltaY), (30,60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255))
        
        cv2.putText(scratchImgCol, "Rad " + str(self.comms.radius), (30,85),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255))
        
        # Draw center of screen
        scratchImgCol = vision.drawCenterRect(scratchImgCol)
                
        return scratchImgCol 
    
    def updateParams(self):
        self.thresParams['lo'] = self.comms.params['loThreshold']
        self.thresParams['hi'] = self.comms.params['hiThreshold']
        self.cannyParams = self.comms.params['cannyParams']
        self.minContourArea = self.comms.params['minContourArea']     

    def normaliseImg(self, img):
        channel = cv2.split(img)
        for i in channel[1]:
            i += 5
        # cv2.normalize(channel[1], channel[1], 0, 255, cv2.NORM_MINMAX)
        cv2.normalize(channel[2], channel[2], 0, 255, cv2.NORM_MINMAX)
        return cv2.merge(channel, img)  
    
def main():
    cv2.namedWindow("Torpedo")
    
    inImg = cv2.imread("torpedo/torpedo1.png")
    from comms import Comms
    detector = TorpedoVision(comms = Comms())
    outImg = detector.gotFrame(inImg)
    
    if outImg is not None: cv2.imshow("Torpedo", outImg)
    cv2.waitKey()
