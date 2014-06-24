import os
import math

import rospy
import cv2
import numpy as np

from bot_common.vision import Vision
from utils.utils import Utils

class BinsVision:
    screen = { 'width': 840, 'height': 680 }
    centerX = screen['width'] / 2
    centerY = screen['height'] / 2

    # Vision parameters
    hsvLoThresh1 = (1, 0, 0)
    hsvHiThresh1 = (25, 255, 255)
    hsvLoThresh2 = (160, 0, 0)
    hsvHiThresh2 = (180, 255, 255)

    loBlueThresh = (100, 1, 0)
    hiBlueThresh = (120, 255, 255)
    minContourArea = 3000

    # Parameters for gray-scale thresholding
    upperThresh = 70
    areaThresh = 4000
    adaptiveCoeff = 3.99
    adaptiveOffset = 0.0

    # Contours of aliens for shape matching
    aliens = {'1a':None, '1b':None, '2a':None, '2b':None,
              '3a':None, '3b':None, '4' :None}

    def __init__(self, comms=None, debugMode=True):
        self.comms = comms
        self.debugMode = debugMode

        for alien in self.aliens:
            self.aliens[alien] = np.load("{}/res/{}.npy".
                                         format(os.path.dirname(__file__),
                                                alien))
            #self.templates = cv2.imread("{}/res/{}.png".
            #                            format(os.path.dirname(__file__),
            #                                   alien))

    def updateParams(self):
        self.hsvLoThresh1 = self.comms.params['hsvLoThresh1']
        self.hsvHiThresh1 = self.comms.params['hsvHiThresh1']
        self.hsvLoThresh2 = self.comms.params['hsvLoThresh2']
        self.hsvHiThresh2 = self.comms.params['hsvHiThresh2']
        self.minContourArea = self.comms.params['minContourArea']
        self.adaptiveCoeff = self.comms.params['adaptiveCoeff']
        self.adaptiveOffset = self.comms.params['adaptiveOffset']
        self.areaThresh = self.comms.params['areaThresh']

    def morphology(self, img):
        # Closing up gaps and remove noise with morphological ops for aliens
        #erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        closeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        #img = cv2.erode(img, erodeEl)
        img = cv2.dilate(img, dilateEl)
        img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, closeEl, iterations=3)

        return img

    def morphology2(self, img):
        # Closing up gaps and remove noise with morphological ops
        erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        closeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        img = cv2.erode(img, erodeEl)
        img = cv2.dilate(img, dilateEl)
        img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, closeEl, iterations=3)

        return img

    def findContourAndBound(self, img, bounded=True, minArea=0.0):
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_NONE)
        if bounded:
            contours = filter(lambda c: cv2.contourArea(c) > minArea,
                              contours)
        return contours

    def match(self, alienContours, centroids, contours):
        """ Match a centroid with a contour if it is inside the contour
            Return dict of alien_contour, centroid, bin contour, class
            and angle of the bin """
        ret = list()
        for centroid in enumerate(centroids):
            for contour in contours:
                if cv2.pointPolygonTest(contour, centroid[1], False) > 0:
                    thisMatch = {'alien': alienContours[centroid[0]],
                                 'centroid':centroid[1],
                                 'contour': contour}
                    thisMatch['class'] = self.classify(thisMatch)
                    thisMatch['angle'] = self.angleFromContour(contour)
                    ret.append(thisMatch)

        return ret

    def angleFromContour(self, contour):
        points = np.array(cv2.cv.BoxPoints(cv2.minAreaRect(contour)))

        edge1 = points[1] - points[0]
        edge2 = points[2] - points[1]

        #Choose the vertical edge
        if cv2.norm(edge1) > cv2.norm(edge2):
            rectAngle = math.degrees(math.atan2(edge1[1], edge1[0]))
        else:
            rectAngle = math.degrees(math.atan2(edge2[1], edge2[0]))

        return rectAngle


    def classify(self, match):
        """ Classify a match -> {alienContour, centroid, contour}
            into an alien category """
        rval = None
        closestMatch = float("inf")
        for alien in self.aliens:
            humatch = cv2.matchShapes(self.aliens[alien], match['alien'],
                                      1, 0)
            if humatch < closestMatch:
                closestMatch = humatch
                rval = alien

        return rval

    def gotFrame(self, img):
        """ Main processing function, should return (retData, outputImg) """
        centroids = list()
        outImg = None
        matches = list()
        retData = {'centroids': centroids, 'matches': matches}

        img = cv2.resize(img, (self.screen['width'], self.screen['height']))
        img = Vision.enhance(img)
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        binImg = cv2.inRange(hsvImg, self.hsvLoThresh1, self.hsvHiThresh1)
        binImg |= cv2.inRange(hsvImg, self.hsvLoThresh2, self.hsvHiThresh2)

        binImg = self.morphology(binImg)

        if self.debugMode:
            outImg1 = cv2.cvtColor(binImg.copy(), cv2.COLOR_GRAY2BGR)
            # Draw the aiming rectangle
            midX = self.screen['width']/2.0
            midY = self.screen['height']/2.0
            maxDeltaX = self.screen['width']*0.03
            maxDeltaY = self.screen['height']*0.03
            cv2.rectangle(outImg1,
                          (int(midX-maxDeltaX), int(midY-maxDeltaY)),
                          (int(midX+maxDeltaX), int(midY+maxDeltaY)),
                          (0, 255, 0), 2)

        scratchImg = binImg.copy()
        alienContours = self.findContourAndBound(scratchImg,
                                                 bounded=True,
                                                 minArea=self.minContourArea)
        #if not contours or len(contours) < 1: return retData, outImg
        sorted(alienContours, key=cv2.contourArea, reverse=True)

        for contour in alienContours:
            moment = cv2.moments(contour, False)
            centroids.append((moment['m10']/moment['m00'],
                              moment['m01']/moment['m00']))

        if self.debugMode:
            for centroid in centroids:
                cv2.circle(outImg1, (int(centroid[0]), int(centroid[1])), 5,
                           (0, 0, 255))

        # Threshold and find contours that represent the black bins
        grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        grayImg = cv2.equalizeHist(grayImg)
        mean = cv2.mean(grayImg)[0]
        lowest = cv2.minMaxLoc(grayImg)[0]
        thVal = min((lowest + mean)/self.adaptiveCoeff + self.adaptiveOffset,
                    self.upperThresh)
        grayImg = cv2.threshold(grayImg, thVal, 255, cv2.THRESH_BINARY_INV)[1]
        grayImg = self.morphology2(grayImg)
        if self.debugMode == True:
            outImg2 = cv2.cvtColor(grayImg.copy(), cv2.COLOR_GRAY2BGR)

        contours = self.findContourAndBound(grayImg, minArea=self.areaThresh)
        sorted(contours, key=cv2.contourArea, reverse=True)

        # Match each alien centroid to a bin
        matches = self.match(alienContours, centroids, contours)
        retData['matches'] = matches

        if self.debugMode:
            for match in matches:
                center = match['centroid']
                cv2.putText(outImg1,
                            match['class'], (int(center[0]), int(center[1])),
                            cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)

                angle = match['angle'] 
                endPt = (int(center[0] + 100*math.cos(math.radians(angle))),
                         int(center[1] + 100*math.sin(math.radians(angle))))
                center = (int(center[0]), int(center[1]))

                #rospy.loginfo("Alien Area: {}".format(cv2.contourArea(match['alien'])))
                Vision.drawRect(outImg1,
                                cv2.cv.BoxPoints(cv2.minAreaRect(match['alien'])))
                Vision.drawRect(outImg2,
                                cv2.cv.BoxPoints(cv2.minAreaRect(match['contour'])))
                cv2.line(outImg2, center, endPt, (0, 255, 0), 2)
                cv2.putText(outImg2,
                            str(Utils.toHeadingSpace(angle)),
                            (int(center[0]), int(center[1])),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            if len(matches) > 0:
                meanX = np.mean(map(lambda c: c[0], centroids))
                meanY = np.mean(map(lambda c: c[1], centroids))
                closest = min(matches,
                              key=lambda m:
                              Utils.distBetweenPoints(m['centroid'],
                                                      (self.centerX, self.centerY)))
                Vision.drawRect(outImg1,
                                cv2.cv.BoxPoints(cv2.minAreaRect(closest['alien'])),
                                color=(0, 255, 255))
                cv2.putText(outImg1,
                            "X", (int(meanX), int(meanY)),
                            cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 0, 0), 2)

        outImg = np.hstack((outImg1, outImg2))
        return retData, outImg

def main():
    rospy.init_node("bins_vision")
    cv2.namedWindow("output")
    img = cv2.imread("bins/res/std_bin.png")
    from comms import Comms
    visionFilter = BinsVision(comms=Comms())
    _, outImg = visionFilter.gotFrame(img)
    if outImg is not None: cv2.imshow("output", outImg)
    cv2.waitKey()
