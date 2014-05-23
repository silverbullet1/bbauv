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
    redParams = {'lo': (110, 0, 0), 'hi': (137, 255, 255),
                 'dilate': (7,7), 'erode': (5,5), 'open': (5,5)}
    
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
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsvImg = cv2.GaussianBlur(hsvImg, ksize=(3, 3), sigmaX=0)
                
        if self.comms.findRedPeg:
            # Threshold red 
            params = self.redParams
        else:
            # Threshold blue 
            params = self.blueParams

        # Perform thresholding
        binImg = cv2.inRange(hsvImg, params['lo'], params['hi'])
        binImg = vision.erodeAndDilateImg(binImg, params)

        # Find contours 
        scratchImg = binImg.copy()
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

        # Find Hough circles
        scratchImgCol = cv2.cvtColor(binImg, cv2.COLOR_GRAY2BGR)
        
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
                            
        # How far the centroid is off the screen center
        self.comms.deltaX = float((vision.screen['width']/2 - self.comms.centroidToPick[0])*1.0/vision.screen['width'])                                                                                                                                          
        self.comms.deltaX = self.comms.deltaX * self.comms.deltaXMult  
        cv2.putText(scratchImgCol, str(self.comms.deltaX), (30,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))
              
        outImg = scratchImgCol
        return outImg
            
def main():
    cv2.namedWindow("Peg Test")
    inImg = cv2.imread("pegs/pegs.png")
    from comms import Comms
    detector = PegsVision(comms = Comms())
    outImg = detector.gotFrame(inImg)
    
    if outImg is not None: cv2.imshow("Peg Test", outImg)
    cv2.waitKey()
    
    