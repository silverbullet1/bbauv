#/usr/bin/env/python 

'''
Vision filter chain for the moving of pegs Task
'''

import math
import numpy as np
import cv2

from utils.utils import Utils
from front_commons.frontCommsVision import frontCommsVision as vision

class PegsVision:
    screen = {'width': 640, 'height': 480}
    
    #Vision parameters - pegs are either red or white
    redParams = {'lo': (110, 0, 0), 'hi': (137, 255, 255),
                 'dilate': (7,7), 'erode': (5,5), 'open': (5,5)}
    
    blueParams = {'lo': (17, 18, 2), 'hi': (20, 255, 255),
                  'dilate': (13,13), 'erode': (5,5), 'open': (5,5)}
    
    minContourArea = 5000
    
    def __init__(self, comms = None, debugMode = True):
        self.debugMode = debugMode
        self.comms = comms
        
    def gotFrame(self, img):
        #Set up parameters
        foundLines = []
        centroid = [0, 0]
        outImg = None
        
        #Preprocessing 
        img = cv2.resize(img, (self.screen['width'], self.screen['height']))
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsvImg = cv2.GaussianBlur(hsvImg, ksize=(3, 3), sigmaX  = 0)
        
        if self.comms.findRedPegs:
            # Threshold red 
            params = redParams
        else: 
            # Threshold blue 
            params = blueParams
        
        # Perform thresholding
        binImg = cv2.inRange(image, params['lo'], params['hi'])
        binImg = vision.erodeAndDilateImg(binImg, params)
        
        # Find contours 
        scratchImg = binImg.copy()
        contours, _ = cv2.findContours(stracthImg, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_NONE)
        contours = filter(lambda c: cv2.contourArea(c) > self.minContourArea, contours)
        sorted(contours, key=cv2.contourArea, reverse=True) # Sort by largest contour 

        centers = []
        radii = []
        for contour in contours:
            # Radius
            radii.append(cv2.boundingRect(contour)[2])
            
            # Circle 
            mu = cv2.moments(contour)
            muArea = mu['m00']
            centers.append((mu['m10']/muArea, mu['m01']/muArea))
        
        # Draw the circles and centers 
        radius = int(np.average(radii)) + 5     # Just a random radius hack
        for center in centers:
            cv2.circle(stratchImg, center, 3, (255, 0, 0), -1)
            cv2.circle(scratchImg, center, radius, (0, 255, 0), 1)
        
        return outImg
            
def main():
    cv2.namedWindow("Peg Test")
    inImg = cv2.imread("pegs/test.jpg")
    inImg = cv2.cvtColor(inImg, cv2.COLOUR_RGB2BGR)
    detector = PegsVision()
    outImg = detector.gotFrame(inImg)
    
    if outImg is not None: cv2.imshow("Peg Test", outImg)
    cv2.waitKey()
    
    