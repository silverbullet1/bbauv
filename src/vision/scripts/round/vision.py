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
    
    redCentroid = None
    blackCentroid = None 
    # Remember centerCentroid is in comms 
        
    def __init__(self, comms = None, debugMode = True):
        self.debugMode = debugMode
        self.comms = comms
        
    def gotFrame (self, img):
        # Set up parameters
        outImg = None
        
        # Preprocessing 
        img = cv2.resize(img, (self.screen['width'], self.screen['height']))
        
        # First threshold red
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsvImg = cv2.GaussianBlur(hsvImg, ksize(3, 3), sigmaX = 0)
        
        # Then threshold black 
    
        # Find the center centroids of the two centroids 
        
        # Move towards centroid 
        
        return outImg 
    
def main():
    cv2.namedWindow("Rounds")
    
    inImg = cv2.imread("round/maneuvering.jpg")
    inImg = cv2.cvtColor(inImg, cv2.COLOR_RGB2BGR)
    detector = RoundVision()
    outImg = detector.gotFrame(inImg)
    
    if outImg is not None: cv2.imshow("Rounds", outImg)
    cv2.waitKey()