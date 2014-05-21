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

    minContourArea = 5000
    
    # Keep track of the previous centroids for matching 
    previousCentroid = []

    def __init__(self, comms = None, debugMode = True):
        self.debugMode = debugMode
        self.comms = comms

    def gotFrame(self, img):
        # Set up parameters
        outImg = None

        # Preprocessing
        img = cv2.resize(img, (self.screen['width'], self.screen['height']))
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

        if self.toBump:
            #Need to maintain the centroid and bump? 
            pass

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

        # Find Hough circles
        circles = cv2.HoughCircles(binImg, cv2.cv.CV_HOUGH_GRADIENT, 1,
                                   minDist=30, param1=100, param2=15,
                                   minRadius = self.circleParams['minRadius'],
                                   maxRadius = self.circleParams['maxRadius'])
        
        # Draw Circles
        if circles is not None:
            self.comms.foundBuoy = True
            circles = np.uint16(np.around(circles))
            for circle in circles[0,:,:]:
                #Draw outer circle
                cv2.circle(scratchImg, (circle[0], circle[1]), circle[2], (255, 255, 0), 2)
                #Draw circle center
                cv2.circle(scratchImg, (circle[0], circle[1]), 2, (255, 0, 255), 3)
                currCentroid.append((circle[0], ciFrontCommsrcle[1]))

        # Check if to bump
        if len(currCentroid) == 3:
            self.comms.toBump = True
            rospy.loginfo("Time to bump... {}".format(colour))
            
            # Append to previous centroids 
            for circle in circles[0,:,:]:
                self.previousCentroid.append((circle[0], circle[1]))
            
            # Compare previous centroid to bump to find corresponding one
            if self.centroidToBump is None:
                self.comms.centroidToBump = (currCentroid[0][0], currCentroid[0][1])
                #print Utils.distBetweenPoints(self.centroidToBump, (0,0))   
            else:
                distToPrevCentroid = []
                for previousCentroid in self.previousCentroid:
                    distToPrevCentroid.append(Utils.distBetweenPoints(
                                              previousCentroid, self.comms.centroidToBump))
                minIndex = distToPrevCentroid.index(min(distToPrevCentroid))
                self.comms.centroidToBump = (distToPrevCentroid[minIndex][0], distToPrevCentroid[minIndex][1])    

        thresImg = scratchImg
        return thresImg

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
