#/usr/bin/env/python 

'''
Torpedo Vision
'''

import math
import numpy as np
import cv2

import rospy

from utils.utils import Utils
from front_commons.frontCommsVision import FrontCommsVision as vision

class TorpedoVision:    
    # Vision parameters
    
    thresParams = {
                    'lo': (0, 200, 0), 'hi': (8, 255, 0), # Jin's parameters
                   'dilate': (5,5), 'erode': (3,3), 'open': (3,3)}

    greenParams = {
                    'lo': (100, 0, 125), 'hi': (230, 85, 255), # Lab params
                    # 'lo': (43,0,0), 'hi': (75,255,150),    # Enhanced params
                   'dilate': (5,5), 'erode': (3,3), 'open': (3,3)
                  }
    
    circleParams = {'minRadius': 10, 'maxRadius': 100}
    houghParams = (79, 17)     #Hough circle parameters 
    
    minContourArea = 600
    
    # For circles 
    previousCentroid = (-1, -1)
    previousRadius = 0

    # For board 
    previousBoardCentroid = (-1, -1)
        
    def __init__(self, comms = None, debugMode = True):
        self.debugMode = debugMode
        self.comms = comms
        
    def gotFrame (self, img):
        # Set up parameters
        outImg = None
        
        allCentroidList = []
        allRadiusList = []
        
        # Preprocessing 
        img = cv2.resize(img, (640, 480))
        
        # img = self.illumMask(img)
        
        # img = self.whiteBal(img)

        # img = self.luminanceRemoval(img)

        # hsvImg = self.toHSV(img)
        
        # Enhance image
        # enhancedImg = self.enhance(hsvImg)

        # return cv2.cvtColor(enhancedImg, cv2.COLOR_HSV2BGR)

        labImg = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        channels = cv2.split(labImg)
        # labImgRGB = cv2.cvtColor(channels[2], cv2.COLOR_GRAY2BGR)

        # Threshold out something
        binImg = self.morphology(cv2.inRange(labImg, 
            self.greenParams['lo'], self.greenParams['hi']))
        kern = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        binImg = cv2.morphologyEx(binImg, cv2.MORPH_OPEN, kern)

        # return cv2.cvtColor(binImg, cv2.COLOR_GRAY2BGR)

        # erodeKern = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        # binImg = cv2.erode(binImg, erodeKern, iterations=2)
        # dilateKern = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        # binImg = cv2.dilate(binImg, dilateKern, iterations=3)

        # Find contours and fill them
        for i in range(4):
            binImgCopy = binImg.copy()
            contours, hierarchy = cv2.findContours(binImgCopy,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_NONE)
            cv2.drawContours(image=binImg, contours=contours, contourIdx=-1, color=(255,255,255), thickness=-1)  
        
        '''
        Detect the board first
        '''

        # Detecting the board 
        # Find the largest contours and make sure its a square
        scratchImgCol = cv2.cvtColor(binImg, cv2.COLOR_GRAY2BGR)

        binImgCopy = binImg.copy()
        contours, hierarchy = cv2.findContours(binImgCopy, 
            cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        contours = filter(lambda c: cv2.contourArea(c) > self.minContourArea,
            contours)
        if contours is None:
            return scratchImgCol

        sorted(contours, key=cv2.contourArea, reverse=True)  

        largestContour = contours[0]
        rect = cv2.minAreaRect(largestContour)
        if largestContour is not None:
            self.comms.foundSomething = True

        ((center_x,center_y),(width,height),angle) = cv2.minAreaRect(largestContour)
        box = cv2.cv.BoxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(scratchImgCol, [box], 0, (0,0,255), 2)

        # Find centroid of rect returned 
        mu = cv2.moments(largestContour)
        muArea = mu['m00']
        tempBoardCentroid = (int(mu['m10']/muArea), int(mu['m01']/muArea))
        tempBoardArea = muArea

        self.comms.boardCentroid = tempBoardCentroid
        self.comms.boardArea = tempBoardArea

        # Dist where centroid of board is off 
        self.comms.boardDeltaX = float((self.comms.boardCentroid[0] - vision.screen['width']/2)*1.0/
                                    vision.screen['width'])
        self.comms.boardDeltaY = float((self.comms.boardCentroid[1] - vision.screen['height']/2)*1.0/
                                    vision.screen['height'])

        cv2.putText(scratchImgCol, "Board Area: " + str(self.comms.boardArea), (410, 30),
                    cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255))        
        cv2.circle(scratchImgCol, self.comms.boardCentroid, 2, (0,0,255), 2)
        cv2.putText(scratchImgCol, "Board X: " + str(self.comms.boardDeltaX), (410, 60),
                    cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255))    
        cv2.putText(scratchImgCol, "Board Y: " + str(self.comms.boardDeltaY), (410, 80),
                    cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255))  

        '''
        Detect the circles 
        ''' 

        # Find Hough circles - used to be edges
        scratchImg = binImg.copy()
        circles = cv2.HoughCircles(scratchImg, cv2.cv.CV_HOUGH_GRADIENT, 1,
                                   minDist=10, param1=self.houghParams[0], 
                                   param2=self.houghParams[1],
                                   minRadius = self.circleParams['minRadius'],
                                   maxRadius = self.circleParams['maxRadius'])      
        if circles is None:
            self.comms.foundCount = self.comms.foundCount + 1
            print "HI"  

            return scratchImgCol
        
        self.comms.foundCircles = True
        circlesSorted = np.array(sorted(circles[0], key=lambda x:x[2], reverse=True))
        
        for circle in circlesSorted:
            circleCentroid = (circle[0], circle[1])

            # Only find circles within the bounding rect
            if self.isInRect(circleCentroid, (center_x, center_y), (width, height)):
                allCentroidList.append(circleCentroid)
                allRadiusList.append(circle[2])
                # Draw Circles
                cv2.circle(scratchImgCol, circleCentroid, circle[2], (255, 255, 0), 2)
                cv2.circle(scratchImgCol, circleCentroid, 2, (255, 0, 255), 3)

        # Centroid resetted
        if self.comms.centroidToShoot is None:
            # Pick the largest circle
            if self.comms.numShoot == 0 or len(circles) < 2:
                self.comms.centroidToShoot = (circlesSorted[0][0], circlesSorted[0][1])
                self.comms.radius = circlesSorted[0][2]
            elif self.comms.numShoot == 1:
                self.comms.centroidToShoot = (circlesSorted[1][0], circlesSorted[1][1])
                self.comms.radius = circlesSorted[1][2]
        else:
            # Find the centroid closest to the previous 
            if len(allCentroidList) != 0:
                for centroid in allCentroidList:
                    distDiff = []
                    distDiff.append(Utils.distBetweenPoints(
                                        self.previousCentroid, centroid))
                    minIndex = distDiff.index(min(distDiff))
                    self.comms.centroidToShoot = allCentroidList[minIndex]
                    self.comms.radius = allRadiusList[minIndex]        
            else:
                # If not then just use the previous one 
                self.comms.centroidToShoot = self.previousCentroid
                self.comms.radius = self.previousRadius
                               
        # Draw centroid to shoot 
        self.previousCentroid = self.comms.centroidToShoot   
        self.previousRadius = self.comms.radius
        cv2.circle(scratchImgCol, self.comms.centroidToShoot, 3, (0, 255, 255), 2)
            
        # How far the centroid is off the screen center
        self.comms.deltaX = float((self.comms.centroidToShoot[0] - vision.screen['width']/2)*1.0/
                                    vision.screen['width'])

        # Draw everything
        cv2.putText(scratchImgCol, "X  " + str(self.comms.deltaX), (30,30), 
                    cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))
        self.comms.deltaY = float((self.comms.centroidToShoot[1] - vision.screen['height']/2)*1.0/
                                  vision.screen['height'])
        cv2.putText(scratchImgCol, "Y  " + str(self.comms.deltaY), (30,60), 
                    cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))
        
        cv2.putText(scratchImgCol, "Rad " + str(self.comms.radius), (30,85),
                    cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))           
        
        # Draw center of screen
        scratchImgCol = vision.drawCenterRect(scratchImgCol)
                
        return scratchImgCol 
    
    def whiteBal(self, img):
        channels = cv2.split(img)
        channels[0] = cv2.equalizeHist(channels[0])
        channels[1] = cv2.equalizeHist(channels[1])
        img = cv2.merge(channels, img)
        img = cv2.bilateralFilter(img, -1, 5, 0.1)
        
        # Morphological operations
        kern = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        return cv2.morphologyEx(img, cv2.MORPH_CLOSE, kern, iterations=1)
    
    def updateParams(self):
        self.greenParams['lo'] = self.comms.params['loThreshold']
        self.greenParams['hi'] = self.comms.params['hiThreshold']
        self.houghParams = self.comms.params['houghParams']
        self.minContourArea = self.comms.params['minContourArea']     

    def normaliseImg(self, img):
        channel = cv2.split(img)
        for i in channel[1]:
            i += 5
        # cv2.normalize(channel[1], channel[1], 0, 255, cv2.NORM_MINMAX)
        cv2.normalize(channel[2], channel[2], 0, 255, cv2.NORM_MINMAX)
        return cv2.merge(channel, img)  
    
    def illuminanceMask(self, img):
        grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        grayImg = cv2.equalizeHist(grayImg)
        return cv2.threshold(grayImg, 200, 255, cv2.THRESH_BINARY)[1]

    def isInRect(self, centroid, rect, size):
        if centroid[0] > (rect[0] - size[0]/2) and \
            centroid[0] < (rect[0] + size[0]/2) and \
            centroid[1] > (rect[1] - size[1]/2) and \
            centroid[1] < (rect[1] + size[1]/2):
            return True
        else:
            return False 

    def morphology(self, img):
        kern = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        return cv2.morphologyEx(img, cv2.MORPH_CLOSE, kern)

    def illuminanceRemoval(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
        channels = cv2.split(img)
        channels[0] = cv2.equalizeHist(channels[0])
        img = cv2.merge(channels, img)
        img = cv2.cvtColor(img, cv2.COLOR_YUV2BGR) 
        return img
     
    def illumMask(self, img):
        illumMask = self.illuminanceMask(img)
        illumMask = cv2.bitwise_not(illumMask)
        img = cv2.bitwise_and(cv2.cvtColor(illumMask, cv2.COLOR_GRAY2BGR), img)
        return img

    def toHSV(self, img):
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsvImg = np.array(hsvImg, dtype=np.uint8)  
        return hsvImg

    def enhance(self, img):
        gauss = cv2.GaussianBlur(hsvImg, ksize=(5,5), sigmaX=9)
        sum = cv2.addWeighted(hsvImg, 1.5, gauss, -0.6, 0)
        enhancedImg = cv2.medianBlur(sum, 3)    
        enhancedImg = cv2.GaussianBlur(enhancedImg, ksize=(5,5), sigmaX=2)
        return enhancedImg

    def camCallback(self, rosImg):
        outImg = self.visionFilter.gotFrame(Utils.rosimg2cv(rosImg))
        if self.canPublish and outImg is not None:
            try:
                self.outPub.publish(Utils.cv2rosimg(outImg))
            except Exception, e:
                pass
        rospy.sleep(rospy.Duration(0.5))

def main():
    cv2.namedWindow("Torpedo")
    
    inImg = cv2.imread("torpedo/torpedo1.png")
    from comms import Comms
    detector = TorpedoVision(comms = Comms())
    outImg = detector.gotFrame(inImg)
    
    if outImg is not None: cv2.imshow("Torpedo", outImg)
    cv2.waitKey()
