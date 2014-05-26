#/usr/bin/env/python 

'''
Vision filter chain for the moving of pegs Task
'''

import math
import numpy as np
import cv2

from utils.utils import Utils
from front_commons.frontCommsVision import FrontCommsVision as vision
import rospy

class PegsVision:
    screen = {'width': 640, 'height': 480}
    
    #Vision parameters - pegs are either red or white

    redParams = {'lo1': (0, 0, 0), 'hi1': (25, 255, 255),
                 'lo2': (167, 0, 0), 'hi2': (188, 255, 255),
                 'dilate': (9,9), 'erode': (3,3), 'open': (3,3)}
    
    blueParams = {'lo': (97, 0, 0), 'hi': (139, 255, 255),
                  'dilate': (13,13), 'erode': (5,5), 'open': (5,5)}
    
    # Not tested yet 
    yellowParams = {'lo': (17, 18, 2), 'hi': (20, 255, 255),
                  'dilate': (13,13), 'erode': (5,5), 'open': (5,5)}
    
    circleParams = {'minRadius': 0, 'maxRadius': 200}
    
    minContourArea = 200
    
    previousCentroids = []
    
    def __init__(self, comms = None, debugMode = True):
        self.debugMode = debugMode
        self.comms = comms
        
    def gotFrame(self, img):
        #Set up parameters
        allCentroidList = []
        allAreaList = []
        self.comms.foundSomething = False 
        
        outImg = None
        
        #Preprocessing 
        #img = vision.preprocessImg(img)    # If need then cut image
        
        # Enhance image
        rawImg = img
        blurImg = cv2.GaussianBlur(rawImg, ksize=(0, 0), sigmaX=10)
        enhancedImg = cv2.addWeighted(rawImg, 2.5, blurImg, -1.5, 0)
    
        hsvImg = cv2.cvtColor(enhancedImg, cv2.COLOR_BGR2HSV)
                
        if self.comms.findRedPeg:
            # Threshold red 
            params = self.redParams
        else:
            # Threshold blue 
            params = self.blueParams

        # Perform thresholding
        binImg1 = cv2.inRange(hsvImg, params['lo1'], params['hi1'])
        #binImg2 = cv2.inRange(hsvImg, params['lo2'], params['hi2'])
        #binImg = cv2.bitwise_or(binImg1, binImg2)
        binImg = vision.erodeAndDilateImg(binImg1, params)
        return binImg

        # Find contours 
        scratchImgCol = cv2.cvtColor(binImg, cv2.COLOR_GRAY2BGR)    # To overlay with centroids
        
        scratchImg = binImg.copy()  # For contours to mess up
        contours, _ = cv2.findContours(scratchImg, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_NONE)
        contours = filter(lambda c: cv2.contourArea(c) > self.minContourArea, contours)
        
        sorted(contours, key=cv2.contourArea, reverse=True) # Sort by largest contour 
        
        # Finding center of yellow board 
#         if self.comms.findYellowBoard and len(contours) > 1:
#             self.comms.foundYellowBoard = True
#             mu = cv2.moments(contours[0])
#             muArea = mu['m00']
#             self.comms.centroidToPick = (mu['m10']/muArea, mu['m01']/muArea)
#             self.comms.areaRect = cv2.contourArea(contours[0])
# 
#         else:

        if self.comms.centering:
            # Find largest contour
            largestContour = contours[0]
            mu = cv2.moments(largestContour)
            muArea = mu['m00']
            self.comms.centroidToPick = (mu['m10']/muArea, mu['m01']/muArea)
            self.comms.areaRect = cv2.minAreaRect(largestContour)

            rospy.loginfo("Area of centroid:{}".format(self.comms.areaRect))
            
            # Draw new centroid
            cv2.circle(scratchImgCol, self.comms.centroidToPick, 3, (0, 255, 255), 2)
            # How far the centroid is off the screen center
            
            self.comms.deltaX = float((vision.screen['width']/2 - self.comms.centroidToPick[0])*1.0/vision.screen['width'])                                                                                                                                          
            cv2.putText(scratchImgCol, str(self.comms.deltaX), (30,30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))
            
            return scratchImgCol

        # Find Hough circles        
        circles = cv2.HoughCircles(binImg, cv2.cv.CV_HOUGH_GRADIENT, 1,
                                   minDist=1, param1=350, param2=23,
                                   minRadius = 0,
                                   maxRadius = self.circleParams['maxRadius'])

        # Check if centroid of contour is inside a circle
        for contour in contours:
            mu = cv2.moments(contour)
            muArea = mu['m00']
            centroid = (mu['m10']/muArea, mu['m01']/muArea)
            
            if circles is None:
                self.comms.foundSomething = False 
                return scratchImgCol
            
            for circle in circles[0,:,:]:
                circleCentroid = (circle[0], circle[1])
#                 print abs((Utils.distBetweenPoints(centroid, circleCentroid))), circle[2]
                if abs((Utils.distBetweenPoints(centroid, circleCentroid))) < circle[2]:
                    self.comms.foundSomething = True
                    # Find new centroid by averaging the centroid and the circle centroid
                    newCentroid = (int(centroid[0]+circleCentroid[0])/2, 
                                   int(centroid[1]+circleCentroid[1])/2)
                    allCentroidList.append(newCentroid)
                    allAreaList.append(cv2.contourArea(contour))
                        
                    # Draw Circles
                    cv2.circle(scratchImgCol, newCentroid, circle[2], (255, 255, 0), 2)
                    cv2.circle(scratchImgCol, newCentroid, 2, (255, 0, 255), 3)
                        
                    break            
                
        # Compare to previous centroids and pick the nth one 
        if len(self.previousCentroids) == 0:
            self.previousCentroids = allCentroidList
            self.previousArea = allAreaList
        else:                
            for previousCentroid in self.previousCentroids:
                for i in range(len(allCentroidList)):
                    distCenter = []
                    distCenter.append(Utils.distBetweenPoints(
                                        previousCentroid, allCentroidList[i]))
                minIndex = distCenter.index(min(distCenter))
                self.previousCentroids[minIndex] = allCentroidList[minIndex]
                self.previousArea[minIndex] = allAreaList[minIndex]
        
        self.comms.centroidToPick = self.previousCentroids[self.comms.count]
        self.comms.areaRect = self.previousArea[self.comms.count]
        rospy.loginfo("Area of centroid:{}".format(self.comms.areaRect))
        
        # Draw new centroid
        cv2.circle(scratchImgCol, self.comms.centroidToPick, 3, (0, 255, 255), 2)
        
        # How far the centroid is off the screen center
        self.comms.deltaX = float((vision.screen['width']/2 - self.comms.centroidToPick[0])*1.0/vision.screen['width'])                                                                                                                                          
        cv2.putText(scratchImgCol, str(self.comms.deltaX), (30,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))
              
        outImg = scratchImgCol
        return outImg
            
def main():
    cv2.namedWindow("Peg Test")
    inImg = cv2.imread("pegs/pegs3.png")
    from comms import Comms
    detector = PegsVision(comms = Comms())
    outImg = detector.gotFrame(inImg)
    
    if outImg is not None: cv2.imshow("Peg Test", outImg)
    cv2.waitKey()
    
    