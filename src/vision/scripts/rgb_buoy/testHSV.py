#!/usr/bin/env/python 

'''
To test out the HSV values for different colours 
'''

import math
import numpy as np
import cv2

#Green
loThres = [53, 0, 0]
hiThres = [90, 147, 255]

img = None
outImg = None

def nothing(x):
    pass

def HueLoChanged(x):
    global loThres
    loThres[0] = cv2.getTrackbarPos("Hue Lo", "test")
    drawImg()
    
def SatLoChanged(x):
    global loThres
    loThres[0] = cv2.getTrackbarPos("Sat Lo", "test")   
    drawImg() 

def ValLoChanged(x):
    global loThres
    loThres[0] = cv2.getTrackbarPos("Val Lo", "test")
    drawImg()

def HueHiChanged(x):
    global hiThres
    hiThres[0] = cv2.getTrackbarPos("Hue Hi", "test")
    drawImg()

def SatHiChanged(x):
    global hiThres
    hiThres[0] = cv2.getTrackbarPos("Sat Hi", "test")
    drawImg()

def ValHiChanged(x):
    global hiThres
    hiThres[0] = cv2.getTrackbarPos("Val Hi", "test")
    drawImg()

def drawImg():
    global img
    global outImg
    global loThres
    global hiThres
    
    outImg = cv2.inRange(img, np.array(loThres), np.array(hiThres))
    
    cv2.namedWindow("img")
    if outImg is not None: cv2.imshow("img", outImg)            


def main():    
    global loThres
    global hiThres
    global img
    global outImg
    
    img = cv2.imread("rgb_buoy/RGB6.jpg")
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    
    img = cv2.resize(img, (640, 480))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img = cv2.GaussianBlur(img, ksize=(3, 3), sigmaX = 0)
    
    cv2.namedWindow("HSV")
    cv2.imshow("HSV", img)
    
    cv2.namedWindow("test")
    cv2.createTrackbar('Hue Lo', "test", loThres[0], 255, HueLoChanged)
    cv2.createTrackbar("Hue Hi", "test", hiThres[0], 255, HueHiChanged)
    cv2.createTrackbar("Sat Lo", "test", loThres[1], 255, SatLoChanged)
    cv2.createTrackbar("Sat Hi", "test", hiThres[1], 255, SatHiChanged)
    cv2.createTrackbar("Val Lo", "test", loThres[2], 255, ValLoChanged)
    cv2.createTrackbar("Val Hi", "test", hiThres[2], 255, ValHiChanged)
    
    drawImg()
    cv2.waitKey()
        
        