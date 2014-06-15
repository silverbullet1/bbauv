#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import cv2
import numpy as np

import sys

bridge = CvBridge()
loThresh = np.array([0, 0, 0])
hiThresh = np.array([255, 255, 255])


def rosimg2cv(ros_img):
    global bridge
    try:
        frame = bridge.imgmsg_to_cv2(ros_img, desired_encoding="bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)

    return frame


def onChange(data):
    global loThresh, hiThresh
    loThresh[0] = cv2.getTrackbarPos("loH", "output")
    hiThresh[0] = cv2.getTrackbarPos("hiH", "output")
    loThresh[1] = cv2.getTrackbarPos("loS", "output")
    hiThresh[1] = cv2.getTrackbarPos("hiS", "output")
    loThresh[2] = cv2.getTrackbarPos("loV", "output")
    hiThresh[2] = cv2.getTrackbarPos("hiV", "output")


def shadesofGray(img):
    inB, inG, inR = cv2.split(img)
    avgR = np.mean(inR)
    avgG = np.mean(inG)
    avgB = np.mean(inB)
    avgGray = np.mean((avgB, avgG, avgR))

    if avgB == 0:
        outB = inB
    else:
        outB = (avgGray/avgB)*inB

    if avgG == 0:
        outG = inG
    else:
        outG = (avgGray/avgG)*inG

    if avgR == 0:
        outR = inR
    else:
        outR = (avgGray/avgR)*inR

    maxRGB = (np.max(outR), np.max(outG), np.max(outB))
    factor = np.max(maxRGB)
    if factor > 1:
        outR = 255*outR/factor
        outG = 255*outG/factor
        outB = 255*outB/factor

    outImg = cv2.merge((np.uint8(outB), np.uint8(outG), np.uint8(outR)))
    return outImg


def illuminanceMask(img):
    grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    grayImg = cv2.equalizeHist(grayImg)
    return cv2.threshold(grayImg, 200, 255, cv2.THRESH_BINARY)[1]


def enhance(img):
    blurImg = cv2.GaussianBlur(img, ksize=(0, 0), sigmaX=10)
    enhancedImg = cv2.addWeighted(img, 2.5, blurImg, -1.5, 0)
    return enhancedImg


def camCallback(rosImg):
    global loThresh, hiThresh
    rawImg = rosimg2cv(rosImg)
    rawImg = cv2.resize(rawImg, (320, 250))
    enhancedImg = enhance(rawImg)

    hsvImg = cv2.cvtColor(enhancedImg, cv2.COLOR_BGR2HSV)
    threshImg = cv2.inRange(hsvImg, loThresh, hiThresh)
    threshImg = cv2.cvtColor(threshImg, cv2.COLOR_GRAY2BGR)

    outImg = np.hstack((enhancedImg, threshImg))
    cv2.imshow("output", outImg)
    cv2.waitKey(5)


def main():
    if len(sys.argv) < 2:
        print "Please specify image topic"
        return

    cv2.namedWindow("output")
    cv2.createTrackbar("loH", "output", 0, 180, onChange)
    cv2.createTrackbar("hiH", "output", 0, 180, onChange)
    cv2.createTrackbar("loS", "output", 0, 255, onChange)
    cv2.createTrackbar("hiS", "output", 0, 255, onChange)
    cv2.createTrackbar("loV", "output", 0, 255, onChange)
    cv2.createTrackbar("hiV", "output", 0, 255, onChange)

    rospy.Subscriber(sys.argv[1], Image, camCallback)

    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("hsv_threshold")
    main()
