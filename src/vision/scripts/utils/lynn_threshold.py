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
    loThresh[0] = cv2.getTrackbarPos("loH", "Trackbar")
    hiThresh[0] = cv2.getTrackbarPos("hiH", "Trackbar")
    loThresh[1] = cv2.getTrackbarPos("loS", "Trackbar")
    hiThresh[1] = cv2.getTrackbarPos("hiS", "Trackbar")
    loThresh[2] = cv2.getTrackbarPos("loV", "Trackbar")
    hiThresh[2] = cv2.getTrackbarPos("hiV", "Trackbar")




def camCallback(rosImg):
    global loThresh, hiThresh
    rawImg = rosimg2cv(rosImg)
    rawImg = cv2.resize(rawImg, (320, 250))
    
    threshImg = cv2.inRange(rawImg, loThresh, hiThresh)
    threshImg = cv2.cvtColor(threshImg, cv2.COLOR_GRAY2BGR)

    outImg = np.hstack((rawImg, threshImg))
    cv2.imshow("output", outImg)
    cv2.waitKey(5)

def main():
    if len(sys.argv) < 2:
        print "Please specify image topic"
        return

    cv2.namedWindow("output")
    cv2.namedWindow("Trackbar")
    cv2.createTrackbar("loH", "Trackbar", 0, 180, onChange)
    cv2.createTrackbar("hiH", "Trackbar", 0, 180, onChange)
    cv2.createTrackbar("loS", "Trackbar", 0, 255, onChange)
    cv2.createTrackbar("hiS", "Trackbar", 0, 255, onChange)
    cv2.createTrackbar("loV", "Trackbar", 0, 255, onChange)
    cv2.createTrackbar("hiV", "Trackbar", 0, 255, onChange)

    rospy.Subscriber(sys.argv[1], Image, camCallback)

    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("lynn_threshold")
    main()
