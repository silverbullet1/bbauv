#/usr/bin/env/python 

import math
import numpy as np
import cv2

import rospy

from utils.utils import Utils
from front_commons.frontCommsVision import FrontCommsVision as vision

class RoundVision:
    screen = {'width': 640, 'height': 480}
    
    # Vision parameters
    redParams = {'lo': (111, 0, 0), 'hi': (187, 255, 255),
                 'dilate': (7,7), 'erode': (5,5), 'open': (5,5)}    
    
    greenParams = {'lo': (39, 0, 0), 'hi': (80, 255, 255),
             'dilate': (7,7), 'erode': (5,5), 'open': (3,3)}  
    
    minContourArea = 5000
    
    # Remember centerCentroid is in comms 
    redCentroid = None
    redArea = None
    greenCentroid = None 
    greenArea = None
        
    def __init__(self, comms = None, debugMode = True):
        self.debugMode = debugMode
        self.comms = comms
        
    def gotFrame (self, img):
        # Set up parameters
        outImg = None
        
        # Preprocessing     
        # img = vision.preprocessImg(img)    # Cuts the image top out if needed
        # Enhance image
        rawImg = img
        blurImg = cv2.GaussianBlur(rawImg, ksize=(0, 0), sigmaX=10)
        enhancedImg = cv2.addWeighted(rawImg, 2.5, blurImg, -1.5, 0)
    
        hsvImg = cv2.cvtColor(enhancedImg, cv2.COLOR_BGR2HSV)

        #hsvImg = cv2.GaussianBlur(hsvImg, ksize=(3,3), sigmaX=0)
        
        # First threshold red
        redImg = self.thresholdRed(hsvImg)
        
        # Then threshold green
        greenImg = self.thresholdGreen(hsvImg)
        #return greenImg
    
        # Find the center centroids of the two centroids 
        if self.redCentroid is not None and \
            self.greenCentroid is not None:
            self.comms.centerCentroid = ((self.redCentroid[0]+self.greenCentroid[1])/2, 
                                         (self.redCentroid[1]+self.greenCentroid[1])/2)
        # Can only see one centroid - move it out of the screen
        elif self.redCentroid is None and self.greenCentroid is not None:
            self.comms.deltaX = (vision.screen['width']-self.greenCentroid[0]) 
        elif self.greenCentroid is None and self.redCentroid is not None:
            self.comms.deltaX = (vision.screen['width']-self.redCentroid[0])
        else:
            self.comms.centerCentroid = (vision.screen['width']/2, vision.screen['height']/2)
                     
        #outImg = redImg | greenImg 
            
        # Find offset from the screen center and make a multiplier for movement
        self.comms.deltaX = (vision.screen['width']/2-self.comms.centerCentroid[0])
        cv2.putText(outImg, str(self.comms.deltaX), (30,30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))   
           
        return outImg 
    
    def thresholdRed(self, image):
        binImg = cv2.inRange(image, self.redParams['lo'], self.redParams['hi'])
        return binImg

        binImg = vision.erodeAndDilateImg(binImg, self.redParams)
        
        # Find contours
        scratchImg = binImg.copy()
        scratchImgCol = cv2.cvtColor(scratchImg, cv2.COLOR_GRAY2BGR)
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
            cv2.circle(scratchImgCol, self.redCentroid, 3, (255, 255, 0), 2)
            points = np.array(cv2.cv.BoxPoints(cv2.minAreaRect(largestContour)))
            
            for i in range(4):
                pt1 = (int(points[i][0]), int(points[i][1]))
                pt2 = (int(points[(i+1)%4][0]), int(points[(i+1)%4][1]))
                cv2.line(scratchImgCol, pt1, pt2, (255, 255, 255))
        
        return scratchImgCol
    
    def thresholdGreen(self, image):
        binImg = cv2.inRange(image, self.greenParams['lo'], self.greenParams['hi'])  
        return binImg
        binImg = vision.erodeAndDilateImg(binImg, self.greenParams)
        
        # Find contours
        scratchImg = binImg.copy()
        scratchImgCol = cv2.cvtColor(scratchImg, cv2.COLOR_GRAY2BGR)
        contours = vision.findAndSortContours(scratchImg)
        
        if len(contours) > 0:
            self.comms.foundGreen = True
            
            # Center of largest contour
            largestContour = contours[0]
            mu = cv2.moments(largestContour, False)
            muArea = mu['m00']
            self.greenCentroid = (int(mu['m10']/ muArea), int(mu['m01']/muArea))
            self.greenArea = cv2.contourArea(largestContour)
            
            # Draw contour and centroid
            cv2.circle(scratchImgCol, self.greenCentroid, 3, (255, 255, 0), 2)
            points = np.array(cv2.cv.BoxPoints(cv2.minAreaRect(largestContour)))
            
            for i in range(4):
                pt1 = (int(points[i][0]), int(points[i][1]))
                pt2 = (int(points[(i+1)%4][0]), int(points[(i+1)%4][1]))
                cv2.line(scratchImgCol, pt1, pt2, (255, 255, 255))
        
        return scratchImgCol
    
def main():
    cv2.namedWindow("Rounds")
    
    inImg = cv2.imread("round/round4.png")
    
    from comms import Comms
    detector = RoundVision(comms=Comms())
    outImg = detector.gotFrame(inImg)
    
    if outImg is not None: cv2.imshow("Rounds", outImg)
    cv2.waitKey()