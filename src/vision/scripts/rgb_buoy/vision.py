#/usr/bin/env/python

import math
import numpy as np
import cv2

import rospy

from utils.utils import Utils
from front_commons.frontCommsVision import FrontCommsVision as vision

class RgbBuoyVision:
    screen = {'width': 640, 'height': 480}

    # Vision parameters
    greenParams = {'lo': (24, 30, 50), 'hi': (111, 255, 255),
                   'dilate': (7,7), 'erode': (5,5), 'open': (5,5)}
    redParams = {'lo': (110, 0, 0), 'hi': (137, 255, 255),
                 'dilate': (7,7), 'erode': (5,5), 'open': (5,5)}
    blueParams = {'lo': (17, 18, 2), 'hi': (20, 255, 255),
                  'dilate': (13,13), 'erode': (5,5), 'open': (5,5)}

    # Hough circle parameters
    circleParams = {'minRadius':0, 'maxRadius': 0 }
    allCentroidList = []
    allAreaList = []

    minContourArea = 5000
    
    # Keep track of the previous centroids for matching 
    previousCentroid = []
    previousArea = []
    
    # For movement 
    deltaXMult = 5.0    

    def __init__(self, comms = None, debugMode = True):
        self.debugMode = debugMode
        self.comms = comms

    def gotFrame(self, img):
        # Set up parameters
        outImg = None

        # Preprocessing
        img = vision.preprocessImg(img)
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsvImg = cv2.GaussianBlur(hsvImg, ksize=(3, 3), sigmaX = 0)

        # Find red image
        redImg = self.threshold(hsvImg, "RED")
        #outImg = redImg

        # Find blue image
        blueImg = self.threshold(hsvImg, "BLUE")
        #outImg = blueImg

        # Find green image
        greenImg = self.threshold(hsvImg, "GREEN")
        #outImg = greenImg

        # Combine images
        outImg = blueImg | redImg | greenImg
        
        # Compare with previous centroids
        if len(self.previousCentroid) == 0:
            self.previousCentroid = self.allCentroidList
            self.previousArea = self.allAreaList
        else: 
            for i in range(len(self.allCentroidList)):
                distPrevCentroid = []
                for previousCentroid in self.previousCentroid:
                    distPrevCentroid.append(Utils.distBetweenPoints(
                                        previousCentroid, self.allCentroidList[i]))
                    minIndex = distPrevCentroid.index(min(distPrevCentroid))
                    self.previousCentroid[minIndex] = self.allCentroidList[i]
                    self.previousArea[minIndex] = self.allAreaList[i]
        
            self.comms.centroidToBump = self.previousCentroid[0]
            self.comms.rectArea = self.previousArea[0] 
            
        # Find difference from screen
        self.comms.deltaX = (self.screen['width']/2-self.comms.centroidToPick[0]) * self.deltaXMult        

        return outImg

    def resetRGBCount(self):
        self.rgbCount['red'] = 0
        self.rgbCount['green'] = 0
        self.rgbCount['blue'] = 0

    def threshold(self, image, colour):
        currCentroid = []

        params = self.getParams(colour)
        binImg = cv2.inRange(image, params['lo'], params['hi'])
        binImg = vision.erodeAndDilateImg(binImg, params)

        scratchImg = binImg.copy()        
        scratchImg = cv2.cvtColor(scratchImg, cv2.COLOR_GRAY2BGR)
        
        contours, _ = cv2.findContours(scratchImg, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_NONE)
        contours = filter(lambda c: cv2.contourArea(c) > self.minContourArea, contours)
        
        sorted(contours, key=cv2.contourArea, reverse=True) # Sort by largest contour area

        # Find Hough circles
        circles = cv2.HoughCircles(binImg, cv2.cv.CV_HOUGH_GRADIENT, 1,
                                   minDist=30, param1=100, param2=15,
                                   minRadius = self.circleParams['minRadius'],
                                   maxRadius = self.circleParams['maxRadius'])
        
        # Check if centroid of contour inside a circle
        self.allCentroidList = []
        self.allAreaList = []
        
        for contour in contours:
            insideCircle = False
            mu = cv2.moments(contour)
            muArea = mu['m00']
            centroid = ((mu['m10']/muArea, mu['m01']/muArea))
            
            for circle in circles[0,:,:]:
                circleCentroid = (circle[0], circle[1])
                if (Utils.distBetweenPoints(centroid, circleCentroid)) < circle[2]:
                    insideCircle = True

                    # Find new centroid by averaging the centroid and the circle centroid
                    self.allCentroidList.append((centroid[0]+circleCentroid[0])/2, 
                                        (centroid[1]+circleCentroid[1])/2)
                    self.allAreaList.append(cv2.minAreaRect(contour))
                    
                    # Draw Circles
                    cv2.circle(scratchImg, (circle[0], circle[1]), circle[2], (255, 255, 0), 2)
                    cv2.circle(scratchImg, (circle[0], circle[1]), 2, (255, 0, 255), 3)
                    
                    break
                
        return scratchImg

    def getParams(self, inColour):
        colours = ["RED", "GREEN", "BLUE"]

        if inColour == colours[0]:
            return self.redParams
        elif inColour == colours[1]:
            return self.greenParams
        else:
            return self.blueParams

def main():
    cv2.namedWindow("test")

    inImg = cv2.imread("rgb_buoy/RGB6.jpg")
    inImg = cv2.cvtColor(inImg, cv2.COLOR_RGB2BGR)
    detector = RgbBuoyVision()
    outImg = detector.gotFrame(inImg)

    if outImg is not None: cv2.imshow("test", outImg)
    cv2.waitKey()
