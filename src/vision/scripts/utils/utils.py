#!/usr/bin/env/python

import rospy
from cv_bridge import CvBridge, CvBridgeError

import math

class Utils():
    bridge = CvBridge()

    @staticmethod
    def rosimg2cv(ros_img):
        try:
            frame = Utils.bridge.imgmsg_to_cv2(ros_img, ros_img.encoding)
        except CvBridgeError as e:
            rospy.logerr(e)

        return frame

    @staticmethod
    def cv2rosimg(cv_img):
        try:
            return Utils.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    @staticmethod
    def normAngle(angle):
        while angle < 0:
            angle += 360
        return angle % 360

    @staticmethod
    def toHeadingSpace(angle):
        return 90 + angle

    @staticmethod
    def invertAngle(angle):
        if angle < 0: return angle + 180
        else: return angle - 180

    #Convert line equation to vector equation
    @staticmethod
    def vectorizeLine(pt, angle):
        rad = angle / 180.0 * math.pi
        u = math.cos(rad)
        v = math.sin(rad)
        return (pt, (u, v))

    #Find line intersections from line vector equations
    @staticmethod
    def findIntersection(line1, line2):
        ((x1, y1), (u1, v1)) = line1
        ((x2, y2), (u2, v2)) = line2

        det = 1.0 / (u2 * v1 - u1 * v2)
        dx, dy = x2 - x1, y2 - y1
        t1 = det * (-v2 * dx + u2 *dy)

        return (x1 + t1*u1, y1 + t1*v1)
