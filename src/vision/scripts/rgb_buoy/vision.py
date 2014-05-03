#/usr/bin/env/python

import math
import numpy as np
import cv2

from utils.utils import Utils

class RgbBuoyVision:
    screen = {'width': 640, 'height': 480}
    
    #Vision parameters
    greenThres = {'lo': (30, 0, 27), 'hi': (90, 147, 255)}
    redThres = {'lo': (110, 0, 0), 'hi': (137, 255, 255)}
    blueThres = {'lo': (100, 150, 0), 'hi': (140, 255, 255)}
    
    minContourArea = 5000
    
    rgbCount = {'red': 0, 'green': 0, 'yellow': 0}
    
    def __init__(self, debugMode = True):
        self.debugMode = debugMode
        
    def gotFrame(self, img):        
        #Set up parameters 
        self.resetRGBCount()
        foundLines = []
        centroid = [0, 0]
        outImg = None
        rectData = {'foundLines': foundLines, 'centroid': centroid}
        
        #Preprocessing
        img = cv2.resize(img, (self.screen['width'], self.screen['height']))
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsvImg = cv2.GaussianBlur(hsvImg, ksize=(3, 3), sigmaX = 0)
                
        #Find red image
        #redImg = self.threshold(img, "RED")
        
        #Find blue image
        #blueImg = self.threshold(img, "BLUE")
        
        #Find green image 
        greenImg = self.threshold(hsvImg, "GREEN")
        outImg = greenImg
        
        #If any count is 3, bump
        
        #Combine images 
        
        return outImg
    
    def resetRGBCount(self):
        self.rgbCount['red'] = 0
        self.rgbCount['green'] = 0
        self.rgbCount['yellow'] = 0
    
    def threshold(self, image, colour):
        loThres, hiThres = self.getThreshVal(colour) 
        binImg = cv2.inRange(image, loThres, hiThres)

        thresImg = binImg 
        return thresImg
    
    def getThreshVal(self, inColour):
        colours = ["RED", "GREEN", "BLUE"]
        
        if inColour == colours[0]:
            return self.redThres['lo'], self.redThres['hi']
        elif inColour == colours[1]:
            return self.greenThres['lo'], self.greenThres['hi']
        else:
            return self.blueThres['lo'], self.blueThres['hi']

def main():
    cv2.namedWindow("test")
    
    inImg = cv2.imread("rgb_buoy/RGB6.jpg")
    inImg = cv2.cvtColor(inImg, cv2.COLOR_RGB2BGR)
    detector = RgbBuoyVision()
    outImg = detector.gotFrame(inImg)
    
    if outImg is not None: cv2.imshow("test", outImg)
    cv2.waitKey()