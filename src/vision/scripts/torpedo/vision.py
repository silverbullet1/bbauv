#/usr/bin/env/python 

import math
import numpy as np
import cv2

import rospy

from utils.utils import Utils
from front_commons.frontCommsVision import FrontCommsVision as vision

class TorpedoVision:    
    # Vision parameters
    greenParams = {'lo': (24, 93, 228), 'hi': (55, 187, 255),
                   'dilate': (7,7), 'erode': (5,5), 'open': (5,5)}
    
    blackParams = {'lo': (87, 26, 0), 'hi': (125, 255, 255), 
                   'dilate': (9,9), 'erode': (3,3), 'open': (3,3)}
    
    circleParams = {'minRadius': 10, 'maxRadius': 200}
    
    minContourArea = 5000
    
    previousCentroid = []
    previousRadius = []
        
    def __init__(self, comms = None, debugMode = True):
        self.debugMode = debugMode
        self.comms = comms
        
    def gotFrame (self, img):
        # Set up parameters
        outImg = None
        
        allCentroidList = []
        allAreaList = []
        
        # Preprocessing 
        img = vision.preprocessImg(img)
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        #hsvImg = cv2.GaussianBlur(hsvImg, ksize=(3,3), sigmaX=0)
        
        # Threshold green and center 
        #greenImg = self.findGreenBoard(hsvImg, params)            
        #outImg = greenImg
        
        # Threshold black
        blackImg = self.findBlackCircles(img)
        outImg = blackImg
        
        #outImg = greenImg | blackImg
        
        return outImg 
    
    def findGreenBoard(self, image):
        binImg = cv2.inRange(image, self.greenParams['lo'], self.greenParams['hi'])
        binImg = vision.erodeAndDilateImg(binImg, self.greenParams)
        
        # Find contours 
        scratchImg = binImg.copy()        
        contours = vision.findAndSortContours(scratchImg)    
        
        if len(contours) > 0:
            self.comms.foundGreenBoard = True 
        
        # Center of largest contour
        largestContour = contours[0]
        mu = cv2.moments(largestContour, False)
        muArea = mu['m00']
        self.comms.greenCentroid = (mu['m10']/muArea, mu['m01']/muArea)
        self.comms.greenArea = cv2.contourArea(largestContour)
        
        # Draw contour and centroid 
        cv2.circle(scratchImg, self.comms.greenCentroid, 3, (255, 255, 0), 2)
        points = np.array(cv2.cv.BoxPoints(cv2.minAreaRect(largestContour)))
        
        for i in range(4):
            pt1 = (int(points[i][0]), int(points[i][1]))
            pt2 = (int(points[(i+1)%4][0]), int(points[(i+1)%4][1]))
            cv2.line(scratchImg, pt1, pt2, (255, 255, 255))
        
        # Find difference with center
        if not self.comms.timeToFindCircles:
            self.comms.deltaX = (vision.screen['width']/2 - self.comms.greenCentroid[0]) * self.comms.deltaXMult
            cv2.putText(contourImg, str(self.comms.deltaX), (30,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))
                
        return scratchImg
    
    def findBlackCircles(self, image):
        allCentroidList = []
        allRadiusList = []
#         grayImg = cv2.cvtColor(image, cv2.cv.CV_BGR2GRAY)
#         grayImg = cv2.resize(grayImg, dsize=(vision.screen['width'], vision.screen['height']))
        
        # Split HSV image and threshold
#         hsv_array = cv2.split(image)
#         color_min = np.array(self.blackParams['lo'],np.uint8)
#         color_max = np.array(self.blackParams['hi'],np.uint8)
#         threshold_image = cv2.inRange(image, color_min, color_max)

        # Threshold and noise removal
#         binImg = cv2.inRange(image, self.blackParams['lo'], self.blackParams['hi'])
#         binImg = vision.erodeAndDilateImg(threshold_image, self.blackParams)  
#         erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, self.blackParams['erode'])
#         dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, self.blackParams['dilate'])            
#         threshold_image = cv2.erode(threshold_image, erodeEl)
#         threshold_image = cv2.dilate(threshold_image, dilateEl)        
        
        # Canny edge detection
        edges = cv2.Canny(image, 150, 350)
                
        scratchImg = edges.copy()
        
        # Find Hough circles

        scratchImgCol = cv2.cvtColor(scratchImg, cv2.COLOR_GRAY2BGR)

        circles = cv2.HoughCircles(scratchImg, cv2.cv.CV_HOUGH_GRADIENT, 1,
                                   minDist=5, param1=350, param2=44,
                                   minRadius = 0,
                                   maxRadius = self.circleParams['maxRadius'])

        if circles is not None:    
            self.comms.foundCircles = True
            for circle in circles[0,:,:]:
                circleCentroid = (circle[0], circle[1])
                allCentroidList.append(circleCentroid)
                allRadiusList.append(circle[2])
                        
                # Draw Circles
                cv2.circle(scratchImgCol, (circle[0], circle[1]), circle[2], (255, 255, 0), 2)
                cv2.circle(scratchImgCol, (circle[0], circle[1]), 2, (255, 0, 255), 3)        
            
#         contours = vision.findAndSortContours(scratchImg)  
        
#         if len(contours) > 0 and circles is not None:
#             self.comms.foundCircles = True   

        # Check if centroid of contour is inside a circle
#         allCentroidList = []
#         allAreaList = []
#         for contour in contours:
#             mu = cv2.moments(contour)
#             muArea = mu['m00']
#             centroid = (mu['m10']/muArea, mu['m01']/muArea)
#             
#             for circle in circles[0,:,:]:
#                 circleCentroid = (circle[0], circle[1])
#                 if abs((Utils.distBetweenPoints(centroid, circleCentroid))) < circle[2]:
#                     # Find new centroid by averaging the centroid and the circle centroid
#                     newCentroid = (int((centroid[0]+circleCentroid[0])/2), 
#                                    int((centroid[1]+circleCentroid[1])/2))
#                     allCentroidList.append(newCentroid)
#                     allAreaList.append(cv2.contourArea(contour))
#                     
#                     # Draw Circles
#                     cv2.circle(scratchImg, newCentroid, circle[2], (255, 255, 0), 2)
#                     cv2.circle(scratchImg, newCentroid, 2, (255, 0, 255), 3)
#                     
#                     break
                
        # Compare to previous centroid and pick the nth one 
        if len(self.previousCentroid) == 0:
            self.previousCentroid = allCentroidList
            self.previousRadius = allRadiusList
        else:         
            for previousCentroid in self.previousCentroid:
                for i in range(len(allCentroidList)):
                    distDiff = []
                    rospy.loginfo(previousCentroid)
                    distDiff.append(abs((Utils.distBetweenPoints(
                                        previousCentroid, allCentroidList[i]))))
                    minIndex = distDiff.index(min(distDiff))
                    self.previousCentroid[minIndex] = distDiff[minIndex]
#                     self.previousArea[minIndex] = allAreaList[minIndex]
            
        self.comms.centroidToShoot = self.previousCentroid[self.comms.numShoot]
        self.comms.radius = self.previousRadius[self.comms.numShoot]
        
        # Overwrite if time to find circles 
#         if self.comms.timeToFindCircles:
#             self.comms.areaRect = previousArea[self.comms.count]

        # How far the centroid is off center
        self.comms.deltaX = (vision.screen['width']/2 - self.comms.centroidToShoot[0])*1.0/vision.screen['width'] 
        self.comms.deltaX = self.comms.deltaX * self.comms.deltaXMult
        cv2.putText(scratchImgCol, str(self.comms.deltaX), (30,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))
        
        scratchImg = cv2.resize(scratchImgCol, dsize=(vision.screen['width'], vision.screen['height']))

        return scratchImg
    
def main():
    cv2.namedWindow("Torpedo")
    
    inImg = cv2.imread("torpedo/torpedo1.png")
    inImg = cv2.cvtColor(inImg, cv2.COLOR_RGB2BGR)
    from comms import Comms
    detector = TorpedoVision(comms = Comms())
    outImg = detector.gotFrame(inImg)
    
    if outImg is not None: cv2.imshow("Torpedo", outImg)
    cv2.waitKey()