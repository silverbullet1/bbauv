import math
import numpy as np
import cv2

from utils.utils import Utils

class BinsVision:
    screen = { 'width': 640, 'height': 480 }

    # Vision parameters
    hsvLoThresh = (100, 0, 0)
    hsvHiThresh = (120, 255, 255)
    minContourArea = 5000

    def __init__(self, comms=None, debugMode=True):
        self.comms = comms
        self.debugMode = debugMode

    # Main processing function, should return (retData, outputImg)
    def gotFrame(self, img):
        pass

def main():
    pass
