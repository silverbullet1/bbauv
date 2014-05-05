#/usr/bin/env/python

import math
import numpy as np
import cv2

from utils.utils import Utils

class RgbBuoyVision:
    screen = {'width': 640, 'height': 480}
    
    #Vision parameters
    greenParams = {'lo': (30, 0, 27), 'hi': (90, 147, 255), 
                   'dilate': (7,7), 'erode': (5,5), 'open': (5,5)}
    redParams = {'lo': (110, 0, 0), 'hi': (137, 255, 255),
                 'dilate': (7,7), 'erode': (5,5), 'open': (5,5)}
    blueParams = {'lo': (18, 16, 2), 'hi': (180, 255, 255),
                  'dilate': (7,7), 'erode': (3,3), 'open': (3,3)}
    
    minContourArea = 5000

    #Whether we should bump the lights 
    rgbCount = {'red': 0, 'green': 0, 'yellow': 0}
    toBump = False
    
    def __init__(self, debugMode = True):
        self.debugMode = debugMode
        
    def gotFrame(self, img):        
        #Set up parameters 
        self.resetRGBCount()
        foundLines = []
        centroid = [0, 0]
        outImg = None
        
        #Preprocessing
        img = cv2.resize(img, (self.screen['width'], self.screen['height']))
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsvImg = cv2.GaussianBlur(hsvImg, ksize=(3, 3), sigmaX = 0)
                
        #Find red image
        _, redImg = self.threshold(hsvImg, "RED")
        
        #Find blue image
        _, blueImg = self.threshold(hsvImg, "BLUE")
        
        #Find green image 
        _, greenImg = self.threshold(hsvImg, "GREEN")
              
        #If any count is 3, bump
        self.checkCountAndBump()
        
        #Combine images 
        outImg = blueImg | redImg | greenImg
        
        return outImg
    
    def resetRGBCount(self):
        self.rgbCount['red'] = 0
        self.rgbCount['green'] = 0
        self.rgbCount['yellow'] = 0
    
    def threshold(self, image, colour):
        centroid = [0, 0]
        rectData = {'foundLines': foundLines, 'centroid': centroid}
        
        params = self.getParams(colour) 
        binImg = cv2.inRange(image, params['lo'], params['hi'])
        binImg = self.erodeAndDilateImg(binImg, params)
        
        scratchImg = binImg.copy()
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, 
                                       cv2.CHAIN_APPROX_NONE)
        contours = filter(lambda c: cv2.contourArea(c) > self.minContourArea, contours)
        
        if len(contours) < 1: return retData, outImg
        
        contourRects = [cv2.minAreaRect(contour) for contour in contours]
        
        thresImg = binImg 
        return thresImg
    
    def getParams(self, inColour):
        colours = ["RED", "GREEN", "BLUE"]
        
        if inColour == colours[0]:
            return self.redParams
        elif inColour == colours[1]:
            return self.greenParams
        else:
            return self.blueParams

    def erodeAndDilateImg(self, image, params):
        erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, params['erode'])
        dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, params['dilate'])
        openEl = cv2.getStructuringElement(cv2.MORPH_RECT, params['open'])
        
        image = cv2.erode(image, erodeEl)
        image = cv2.dilate(image, dilateEl)
        image = cv2.morphologyEx(image, cv2.MORPH_OPEN, openEl)
        
        return image

    def checkCountAndBump(self):
        for key in self.rgbCount:
            if self.rgbCount[key] == 3:
                self.toBump = True
                rospy.loginfo("Bumping...{}".format(key))
                #TODO: Input bump code here 

def main():
    cv2.namedWindow("test")
    
    inImg = cv2.imread("rgb_buoy/RGB5.jpg")
    inImg = cv2.cvtColor(inImg, cv2.COLOR_RGB2BGR)
    detector = RgbBuoyVision()
    outImg = detector.gotFrame(inImg)
    
    if outImg is not None: cv2.imshow("test", outImg)
    cv2.waitKey()