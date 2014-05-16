'''
Common methods for front camera vision
'''

import cv2
import math
import numpy

class FrontCommsVision():
    screen = {'width': 640, 'height': 480}
    
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
    def preProcessing(self, image):
        img = cv2.resize(img, (self.screen['width'], self.screen['height']))
        img = self.enhanceImg(img)
        return img 
        
    # Enhance img by addweighted
    def enhanceImg(self, img):
        enhancedImg = cv2.GaussianBlur(img, ksize=(0, 0), sigmaX=10)
        enhancedImg = cv2.addWeighted(img, 2.5, enhancedImg, -1.5, 0)
        return enhancedImg