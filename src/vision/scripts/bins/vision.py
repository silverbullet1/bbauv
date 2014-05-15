import math
import numpy as np
import cv2

from utils.utils import Utils

class BinsVision:
    screen = { 'width': 640, 'height': 480 }

    # Vision parameters
    hsvLoThresh1 = (0, 5, 5)
    hsvHiThresh1 = (35, 250, 250)
    hsvLoThresh2 = (165, 5, 5)
    hsvHiThresh2 = (180, 250, 250)
    minContourArea = 5000

    def __init__(self, comms=None, debugMode=True):
        self.comms = comms
        self.debugMode = debugMode

    def morphology(self, img):
        # Closing up gaps and remove noise with morphological ops
        erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        openEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        img = cv2.erode(img, erodeEl)
        img = cv2.dilate(img, dilateEl)
        img = cv2.morphologyEx(img, cv2.MORPH_OPEN, openEl)

        return img

    def findContourAndBound(self, img, bounded=True):
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_NONE)
        if bounded:
            contours = filter(lambda c: cv2.contourArea(c) > self.minContourArea,
                              contours)
        return contours

    def enhance(self, img):
        enhancedImg = cv2.GaussianBlur(img, ksize=(0, 0), sigmaX=10)
        enhancedImg = cv2.addWeighted(img, 2.5, enhancedImg, -1.5, 0)
        return enhancedImg

    # Main processing function, should return (retData, outputImg)
    def gotFrame(self, img):
        foundRects = list()
        centroids = list()
        outImg = None
        retData = {'foundRects': foundRects, 'centroids': centroids}

        img = cv2.resize(img, (self.screen['width'], self.screen['height']))
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        binImg = cv2.inRange(hsvImg, self.hsvLoThresh1, self.hsvHiThresh1)
        binImg |= cv2.inRange(hsvImg, self.hsvLoThresh2, self.hsvHiThresh2)

        binImg = self.morphology(binImg)

        scratchImg = binImg.copy()
        contours = self.findContourAndBound(scratchImg)
        if not contours or len(contours) < 1: return retData, outImg
        sorted(contours, key=cv2.contourArea, reverse=True)

        contourRects = [cv2.minAreaRect(c) for c in contours]
        for contour in contours:
            moment = cv2.moments(contour, False)
            centroids.append((moment['m10']/moment['m00'],
                              moment['m01']/moment['m00']))

def main():
    pass
