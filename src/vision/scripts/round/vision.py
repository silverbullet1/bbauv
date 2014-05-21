#/usr/bin/env/python 

import math
import numpy as np
import cv2

import rospy

from utils.utils import Utils

class RoundVision:
    screen = {'width': 640, 'height': 480}
    
    # Vision parameters
    redParams = {'lo': (110, 0, 0), 'hi': (137, 255, 255),
                 'dilate': (7,7), 'erode': (5,5), 'open': (5,5)}    
    
    blackParams = {'lo': 30, 'hi': 70,
             'dilate': (7,7), 'erode': (5,5), 'open': (3,3)}  
    
    minContourArea = 5000
    
    # Remember centerCentroid is in comms 
    redCentroid = None
    redArea = None
    blackCentroid = None 
    blackArea = None
        
    def __init__(self, comms = None, debugMode = True):
        self.debugMode = debugMode
        self.comms = comms
        
    def gotFrame (self, img):
        # Set up parameters
        outImg = None
        
        # Preprocessing 
        img = vision.preprocessImg(img)
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsvImg = cv2.GaussianBlur(hsvImg, ksize=(3,3), sigmaX=0)
        
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsvImg = cv2.GaussianBlur(hsvImg, ksize(3, 3), sigmaX = 0)
        
        # First threshold red
        redImg = self.thresholdRed(hsvImg)
        
        # Then threshold black 
        blackImg = self.thresholdBlack(hsvImg)
    
        # Find the center centroids of the two centroids 
        if self.redCentroid is not None and \
            self.blackCentroid is not None:
            self.comms.centerCentroid = ((self.redCentroid[0]+self.blackCentroid[1])/2, 
                                         (self.redCentroid[1]+self.blackCentroid[1])/2)
        elif self.redCentroid is None and self.blackCentroid is not None:
            pass
        elif self.blackCentroid is None and self.redCentroid is not None:
            pass        
        else:
            self.comms.centerCentroid = (vision.screen['width']/2, vision.screen['height']/2)
        
        # Find offset from the screen center and make a multiplier for movement
        self.comms.deltaX = (vision.screen['width']/2-self.comms.centerCentroid[0]) * self.comms.deltaXMult
        
        outImg = redImg | blackImg 
           
        return outImg 
    
    def thresholdRed(self, image):
        binImg = cv2.inRange(image, self.redParams['lo'], self.redParams['hi'])
        binImg = vision.erodeAndDilateImg(binImg, self.redParams)
        
        # Find contours
        scratchImg = binImg.copy()
        contours = vision.findAndSortContours(scratchImg)
        
        if len(contours) > 0:
            self.comms.foundRed = True
            
        # Center of largest contour
        largestContour = contours[0]
        mu = cv2.moments(largestContour, False)
        muArea = mu['m00']
        self.redCentroid = (mu['m10']/ muArea, mu['m01']/muArea)
        self.redArea = cv2.contourArea(largestContour)
        
        # Draw contour and centroid
        cv2.circle(scratchImg, self.redCentroid, 3, (255, 255, 0), 2)
        points = np.array(cv2.cv.BoxPoints(cv2.minAreaRect(largestContour)))
        
        for i in range(4):
            pt1 = (int(points[i][0]), int(points[i][1]))
            pt2 = (int(points[(i+1)%4][0]), int(points[(i+1)%4][1]))
            cv2.line(scratchImg, pt1, pt2, (255, 255, 255))
        
        return scratchImg
    
def main():
    cv2.namedWindow("Rounds")
    
    inImg = cv2.imread("round/maneuvering.jpg")
    inImg = cv2.cvtColor(inImg, cv2.COLOR_RGB2BGR)
    detector = RoundVision()
    outImg = detector.gotFrame(inImg)
    
    if outImg is not None: cv2.imshow("Rounds", outImg)
    cv2.waitKey()