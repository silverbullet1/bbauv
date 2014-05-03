import math
import numpy as np
import cv2

from utils.utils import Utils

class RgbBuoyVision:
    #BLAH
    pass

def main():
    cv2.namedWindow("test")
    inImg = cv2.imread("rgb_buoy/test1.jpg")
    inImg = cv2.cvtColor(inImg, cv2.COLOR_RGB2BGR)
    detector = RgbBuoyVision()
    _, outImg = detector.gotFrame(inImg)
    if outImg is not None: cv2.imshow("test", outImg)
    cv2.waitKey()