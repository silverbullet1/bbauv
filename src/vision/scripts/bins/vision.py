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

    def findContourAndBound(self, img):
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_NONE)
        contours = filter(lambda c: cv2.contourArea(c) > self.minContourArea,
                          contours)
        return contours

    # Main processing function, should return (retData, outputImg)
    def gotFrame(self, img):
        foundRects = []
        outImg = None
        retData = {'foundRects': foundRects}

        img = cv2.resize(img, (self.screen['width'], self.screen['height']))
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        binImg = cv2.inRange(hsvImg, self.hsvLoThresh1, self.hsvHiThresh1)
        binImg |= cv2.inRange(hsvImg, self.hsvLoThresh2, self.hsvHiThresh2)

        binImg = self.morphology(binImg)

        scratchImg = binImg.copy()
        contours = self.findContourAndBound(scratchImg)
        if not contours or len(contours) < 1: return retData, outImg

def main():
    pass
