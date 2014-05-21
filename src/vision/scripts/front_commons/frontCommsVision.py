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
    def erodeAndDilateImg(image, params):
        erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, params['erode'])
        dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, params['dilate'])
        openEl = cv2.getStructuringElement(cv2.MORPH_RECT, params['open'])

        image = cv2.erode(image, erodeEl)
        image = cv2.dilate(image, dilateEl)
        image = cv2.morphologyEx(image, cv2.MORPH_OPEN, openEl)

        return image
    
    # For preprocessing 
    @staticmethod
    def preprocessImg(image):
        image = cv2.resize(image, (self.screen['width'], self.screen['height']))
        enhancedImg = cv2.GaussianBlur(image, ksize=(0, 0), sigmaX=10)
        enhancedImg = cv2.addWeighted(image, 2.5, enhancedImg, -1.5, 0)
        return enhancedImg 
    
    # Contour finding and sorting
    @staticmethod
    def findAndSortContours(image):
        contours, hierachy = cv2.findContours(scratchImg, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        contours = filter(lambda c: cv2.contourArea(c) > self.minContourArea, contours)
        sorted(contours, key=cv2.contourArea, reverse=True)
        return contours               