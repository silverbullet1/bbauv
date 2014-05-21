import math
import numpy as np
import cv2

from utils.utils import Utils

class PickupVision:
    screen = { 'width': 640, 'height': 480 }

    # Vision parameters
    hsvLoThresh1 = (1, 0, 0)
    hsvHiThresh1 = (30, 255, 255)
    hsvLoThresh2 = (165, 0, 0)
    hsvHiThresh2 = (180, 255, 255)
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
        outImg = None
        centroids = list()
        rval = {'centroids':centroids}

        img = cv2.resize(img, (self.screen['width'], self.screen['height']))
        img = self.enhance(img)
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        binImg = cv2.inRange(hsvImg, self.hsvLoThresh1, self.hsvHiThresh1)
        binImg |= cv2.inRange(hsvImg, self.hsvLoThresh2, self.hsvHiThresh2)

        if self.debugMode:
            outImg = cv2.cvtColor(binImg.copy(), cv2.COLOR_GRAY2BGR)

        contours = self.findContourAndBound(binImg.copy(), bounded=True)
        sorted(contours, key=cv2.contourArea, reverse=True)
        for contour in contours:
            moment = cv2.moments(contour, False)
            centroids.append((moment['m10']/moment['m00'],
                              moment['m01']/moment['m00']))

        if self.debugMode:
            for centroid in centroids:
                cv2.circle(outImg, (int(centroid[0]),int(centroid[1])),
                           5, (0, 0, 255))

        return rval, outImg

def main():
    pass
