#/usr/bin/env/python 

import math
import numpy as np
import cv2

import rospy

from utils.utils import Utils
from front_commons.frontCommsVision import FrontCommsVision as vision

class RoundVision:    
    # Vision parameters

        
    def __init__(self, comms = None, debugMode = True):
        self.debugMode = debugMode
        self.comms = comms
        
    def gotFrame (self, img):
        # Set up parameters
        outImg = None
        
        # Preprocessing 
        img = cv2.resize(img, (self.screen['width'], self.screen['height']))

        
        return outImg 
    
def main():
    cv2.namedWindow("Torpedo")
    
    inImg = cv2.imread("torpedo/torpedo.jpg")
    inImg = cv2.cvtColor(inImg, cv2.COLOR_RGB2BGR)
    detector = RoundVision()
    outImg = detector.gotFrame(inImg)
    
    if outImg is not None: cv2.imshow("Torpedo", outImg)
    cv2.waitKey()