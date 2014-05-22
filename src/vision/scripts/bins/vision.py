import os

import cv2
import numpy as np

from bot_common.vision import Vision

class BinsVision:
    screen = { 'width': 640, 'height': 480 }

    # Vision parameters
    hsvLoThresh1 = (1, 0, 0)
    hsvHiThresh1 = (20, 255, 255)
    hsvLoThresh2 = (165, 0, 0)
    hsvHiThresh2 = (180, 255, 255)
    minContourArea = 5000

    # Parameters for gray-scale thresholding
    upperThresh = 70
    areaThresh = 10000

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

    def morphology(self, img):
        # Closing up gaps and remove noise with morphological ops
        erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        openEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        img = cv2.erode(img, erodeEl)
        img = cv2.dilate(img, dilateEl)
        img = cv2.morphologyEx(img, cv2.MORPH_OPEN, openEl)

        return img

    def findContourAndBound(self, img, bounded=True, minArea=0.0):
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_NONE)
        if bounded:
            contours = filter(lambda c: cv2.contourArea(c) > minArea,
                              contours)
        return contours

    def enhance(self, img):
        enhancedImg = cv2.GaussianBlur(img, ksize=(0, 0), sigmaX=10)
        enhancedImg = cv2.addWeighted(img, 2.5, enhancedImg, -1.5, 0)
        return enhancedImg

    def match(self, alienContours, centroids, contours):
        """ Match a centroid with a contour if it is inside the contour
            Return triplet of alien_contour, centroid and contour """
        ret = list()
        for centroid in enumerate(centroids):
            for contour in contours:
                if cv2.pointPolygonTest(contour, centroid[1], False) > 0:
                    ret.append({'alien':alienContours[centroid[0]],
                                'centroid':centroid[1],
                                'contour': contour})

        return ret

    def classify(self, match):
        """ Classify a match -> {alienContour, centroid, contour}
            into an alien category """
        rval = None
        closestMatch = float("inf")
        for alien in self.aliens:
            humatch = cv2.matchShapes(alien, match['alien'])
            if humatch < closestMatch:
                closestMatch = humatch
                rval = alien

        return rval

    def gotFrame(self, img):
        """ Main processing function, should return (retData, outputImg) """
        centroids = list()
        outImg = None
        matches = list()
        classes = None
        retData = {'centroids': centroids, 'matches': matches,
                   'classes': classes}

        img = cv2.resize(img, (self.screen['width'], self.screen['height']))
        img = self.enhance(img)
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        binImg = cv2.inRange(hsvImg, self.hsvLoThresh1, self.hsvHiThresh1)
        binImg |= cv2.inRange(hsvImg, self.hsvLoThresh2, self.hsvHiThresh2)

        binImg = self.morphology(binImg)

        if self.debugMode:
            outImg1 = cv2.cvtColor(binImg.copy(), cv2.COLOR_GRAY2BGR)

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
        outImg2 = cv2.cvtColor(grayImg.copy(), cv2.COLOR_GRAY2BGR)
        mean = cv2.mean(grayImg)[0]
        lowest = cv2.minMaxLoc(grayImg)[0]
        thVal = min((lowest + mean)/3.99, self.upperThresh)
        grayImg = cv2.threshold(grayImg, thVal, 255, cv2.THRESH_BINARY_INV)[1]

        contours = self.findContourAndBound(grayImg, minArea=self.areaThresh)
        sorted(contours, key=cv2.contourArea, reverse=True)
        contourRects = [cv2.cv.BoxPoints(cv2.minAreaRect(contour))
                        for contour in contours]

        if self.debugMode:
            for rect in contourRects:
                Vision.drawRect(outImg2, rect)

        # Match each alien centroid to a bin
        matches = self.match(alienContours, centroids, contours)
        retData['matches'] = matches

        # Classify each alien
        classes = [self.classify(match) for match in retData['matches']]
        retData['classes'] = classes

        outImg = np.vstack((outImg1, outImg2))
        return retData, outImg

def main():
    cv2.namedWindow("output")
    img = cv2.imread("bins/bins_layout.jpg")
    from comms import Comms
    visionFilter = BinsVision(comms=Comms())
    _, outImg = visionFilter.gotFrame(img)
    if outImg is not None: cv2.imshow("output", outImg)
    cv2.waitKey()
