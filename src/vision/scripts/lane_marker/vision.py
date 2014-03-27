import roslib; roslib.load_manifest('vision')
import rospy

import math
import numpy as np
import cv2

from utils.utils import Utils

class LaneMarkerVision:
    # Vision parameters
    hsvThreshold = { 'loH': 19, 'hiH': 54,
                     'loS': 58, 'hiS': 128,
                     'loV': 0 , 'hiV': 255 }

    # Convert line equation to vector equation
    def vectorizeLine(pt, angle):
        rad = angle / 180.0 * math.pi
        u = math.cos(rad)
        v = math.sin(rad)
        return (pt, (u, v))

    # Find line intersection from line vector equation
    def findIntersection(line1, line2):
        ((x1, y1), (u1, v1)) = line1
        ((x2, y2), (u2, v2)) = line2
        det = 1.0 / (u2 * v1 - u1 * v2)
        dx, dy = x2 - x1, y2 - y1
        t1 = det * (-v2 * dx + u2 * dy)
        t2 = det * (-v1 * dx + u1 * dy)
        return (t1, t2)

    # Main processing function, should return (retData, outputImg)
    def gotFrame(self, rosImage):
        img = Utils.rosimg2cv(rosImage)

def main():
    print "This is vision"
