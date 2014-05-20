'''
Common methods for front camera vision
'''

import cv2
import math
import numpy

class FrontCommsVision():
    screen = { 'width': 640, 'height': 480 }
    
    # For morphological operations 
    @staticmethod
    def erodeAndDilateImg(self, image, params):
        erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, params['erode'])
        dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, params['dilate'])
        openEl = cv2.getStructuringElement(cv2.MORPH_RECT, params['open'])

        image = cv2.erode(image, erodeEl)
        image = cv2.dilate(image, dilateEl)
        image = cv2.morphologyEx(image, cv2.MORPH_OPEN, openEl)

        return image
    
    # For preprocessing 
    @staticmethod
    def preprocessImg(self, image):
        