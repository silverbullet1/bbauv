#/usr/bin/env/python 

import math
import numpy as np
import cv2

import rospy

from utils.utils import Utils
from front_commons.frontCommsVision import FrontCommsVision as vision

class RoundVision:    
    # Vision parameters
    greenParams = {'lo': (24, 30, 50), 'hi': (111, 255, 255),
                   'dilate': (7,7), 'erode': (5,5), 'open': (5,5)}
    
    blackParams = {'lo': 30, 'hi': 70, 'dilate': (7,7), 'erode': (5,5), 'open': (5,5)}
    
    circleParams = {'minRadius': 0, 'maxRadius': 0}
    
    minContourArea = 5000
    
    previousCentroid = []
    previousArea = []
        
    def __init__(self, comms = None, debugMode = True):
        self.debugMode = debugMode
        self.comms = comms
        
    def gotFrame (self, img):
        # Set up parameters
        outImg = None
        
        allCentroidList = []
        allAreaList = []
        
        # Preprocessing 
        img = vision.preprocessImg(img)
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsvImg = cv2.GaussianBlur(hsvImg, ksize=(3,3), sigmaX=0)
        
        # Threshold green and center 
        greenImg = self.findGreenBoard(hsvImg, params)            
        
        # Threshold black
        blackImg = self.findBlackCircles(hsvImg)
        
        outImg = greenImg | blackImg
        
        return outImg 
    
    def findGreenBoard(self, image):
        binImg = cv2.inRange(image, self.greenParams['lo'], self.greenParams['hi'])
        binImg = vision.erodeAndDilateImg(binImg, self.greenParams)
        
        # Find contours 
        scratchImg = binImg.copy()        
        contours = vision.findAndSortContours(scratchImg)    
        
        if len(contours) > 0:
            self.comms.foundGreenBoard = True 
        
        # Center of largest contour
        largestContour = contours[0]
        mu = cv2.moments(largestContour, False)
        muArea = mu['m00']
        self.comms.greenCentroid = (mu['m10']/muArea, mu['m01']/muArea)
        self.comms.greenArea = cv2.contourArea(largestContour)
        
        # Draw contour and centroid 
        cv2.circle(scratchImg, self.comms.greenCentroid, 3, (255, 255, 0), 2)
        points = np.array(cv2.cv.BoxPoints(cv2.minAreaRect(largestContour)))
        
        for i in range(4):
            pt1 = (int(points[i][0]), int(points[i][1]))
            pt2 = (int(points[(i+1)%4][0]), int(points[(i+1)%4][1]))
            cv2.line(scratchImg, pt1, pt2, (255, 255, 255))
        
        # Find difference with center
        if not self.comms.timeToFindCircles:
            self.comms.deltaX = (vision.screen['width']/2 - self.comms.greenCentroid[0]) * self.comms.deltaXMult
                
        return scratchImg
    
    def findBlackCircles(self, image):
        grayImg = cv2.cvtColor(image, cv2.cvtColor.CV_BGR2GRAY)
        grayImg = cv2.resize(grayImg, dsize=(vision.screen['width'], vision.screen['height']))
        
        # Calcuate adaptive threshold value
        mean = cv2.mean(grayImg)[0]
        lowest = cv2.minMaxLoc(grayImg)[0]
        self.blackThVal = min((mean+lowest)/ 3.99, self.blackParams['hi'])
        rospy.logdebug(self.blackThVal)    
        
        # Threshold and noise removal
        grayImg = cv2.threshold(grayImg, self.blackThVal, 255, cv2.THRESH_BINARY_INV)[1]
        grayImg = vision.erodeAndDilateImg(grayImg, self.blackParams)
        
        # Find contours 
        scratchImg = grayImg.copy()
        contours = vision.findAndSortContours(scratchImg)  
        
        if self.comms.timeToFindCircles:
            if len(contours) > 0:
                self.comms.foundCircles = True   
        
        # Find Hough circles
        circles = cv2.HoughCircles(scratchImg, cv2.cv.CV_HOUGH_GRADIENT, 1,
                                   minDist=30, param1=100, param2=15,
                                   minRadius = self.circleParams['minRadius'],
                                   maxRadius = self.circleParams['maxRadius'])
        
        # Check if centroid of contour is inside a circle
        allCentroidList = []
        allAreaList = []
        for contour in contours:
            mu = cv2.moments(contour)
            muArea = mu['m00']
            centroid = (mu['m10']/muArea, mu['01']/muArea)
            
            for circle in circles[0,:,:]:
                circleCentroid = (circle[0], circle[1])
                if abs((Utils.distBetweenPoints(centroid, circleCentroid))) < circle[2]:
                    # Find new centroid by averaging the centroid and the circle centroid
                    newCentroid = ((centroid[0]+circleCentroid[0])/2, (centroid[1]+circleCentroid[1])/2)
                    allCentroidList.append(newCentroid)
                    allAreaList.append(cv2.contourArea(contour))
                    
                    # Draw Circles
                    cv2.circle(scratchImg, newCentroid, circle[2], (255, 255, 0), 2)
                    cv2.circle(scratchImg, newCentroid, 2, (255, 0, 255), 3)
                    
                    break
                
        # Compare to previous centroid and pick the nth one 
        if len(self.previousCentroid) == 0:
            self.previousCentroid = allCentroidList
            self.previousArea = allAreaList
        else:         
            for previousCentroid in self.previousCentroid:
                for i in range(len(allCentroidList)):
                    distDiff = []
                    distDist.append(abs((Utils.distBetweenPoints(
                                        previousCentroid, centers[i]))))
                    minIndex = distDiff.index(min(distDiff))
                    self.previousCentroid[minIndex] = (distCenter[minIndex][0], distCenter[minIndex][1])
                    self.previousArea[minIndex] = allAreaList[minIndex]
            
        self.comms.centroidToShoot = previousCentroid[self.comms.count]
        # Overwrite if time to find circles 
        if self.comms.timeToFindCircles:
            self.comms.areaRect = previousArea[self.comms.count]
            # How far the centroid is off center
            self.comms.deltaX = (vision.screen['width']/2 - self.comms.centroidToShoot[0]) * self.deltaXMult
        
        return scratchImg
    
def main():
    cv2.namedWindow("Torpedo")
    
    inImg = cv2.imread("torpedo/torpedo.jpg")
    inImg = cv2.cvtColor(inImg, cv2.COLOR_RGB2BGR)
    detector = RoundVision()
    outImg = detector.gotFrame(inImg)
    
    if outImg is not None: cv2.imshow("Torpedo", outImg)
    cv2.waitKey()