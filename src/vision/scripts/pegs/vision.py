#/usr/bin/env/python 

'''
Vision filter chain for the moving of pegs Task
'''

import math
import numpy as np
import cv2

from utils.utils import Utils

class PegsVision:
    screen = {'width': 640, 'height': 480}
    
    #Vision parameters - pegs are either red or white
    redParams = {'lo': (110, 0, 0), 'hi': (137, 255, 255),
                 'dilate': (7,7), 'erode': (5,5), 'open': (5,5)}
    
    blueParams = {'lo': (17, 18, 2), 'hi': (20, 255, 255),
                  'dilate': (13,13), 'erode': (5,5), 'open': (5,5)}
    
    minContourArea = 5000
    
    def __init__(self, debugMode = True):
        self.debugMode = debugMode
        self.searchRedPeg = True
        
    def gotFrame(self, img):
        #Set up parameters
        foundLines = []
        centroid = [0, 0]
        outImg = None
        
        #Preprocessing 
        img = cv2.resize(img, (self.screen['width'], self.screen['height']))
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsvImg = cv2.GaussianBlur(hsvImg, ksize=(3, 3), sigmaX  = 0)
            
def main():
    cv2.namedWindow("Peg Test")
    inImg = cv2.imread("pegs/test.jpg")
    inImg = cv2.cvtColor(inImg, cv2.COLOUR_RGB2BGR)
    detector = PegsVision()
    outImg = detector.gotFrame(inImg)
    
    if outImg is not None: cv2.imshow("Peg Test", outImg)
    cv2.waitKey()
    
    