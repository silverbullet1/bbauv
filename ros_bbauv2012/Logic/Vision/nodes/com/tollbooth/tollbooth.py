#!/usr/bin/env python2
'''
Code to identify RoboSub tollbooth
'''

import roslib; roslib.load_manifest('Vision')
import rospy
from sensor_msgs.msg import Image
from bbauv_msgs.msg import compass_data
from cv_bridge import CvBridge, CvBridgeError

import random
import numpy as np
import cv2

COLOURS = ['red', 'blue', 'yellow', 'green']

def calcCentroid(contour):
    m = cv2.moments(contour)
    return (m['m10']/m['m00'], m['m01']/m['m00'])

# Tollbooth detector class
class TollboothDetector:
    # Convert a ROS Image to the Numpy matrix used by cv2 functions
    def rosimg2cv(self, ros_image):
        # Convert from ROS Image to old OpenCV image
        frame = self.cvbridge.imgmsg_to_cv(ros_image, ros_image.encoding)
        # Convert from old OpenCV image to Numpy matrix
        return np.array(frame, dtype=np.uint8) #TODO: find out actual dtype


    def __init__(self, params, camdebug=None):
        self.cvbridge = CvBridge()
        self.params = params
        self.camdebug = camdebug
        self.DEBUG = camdebug is not None and camdebug.debugOn

        self.target = 'all'
        self.history = []

        # Initial state
        self.heading = 0.0
        self.regionCount = 0

        self.cvimg = None


    # Callback for subscribing to Image topic
    def gotRosFrame(self, rosImage):
        cvimg = self.rosimg2cv(rosImage)
        self.gotFrame(cvimg)

    # Function that gets called after conversion from ROS Image to OpenCV image
    def gotFrame(self, cvimg):
        imghsv = cv2.cvtColor(cvimg, cv2.cv.CV_BGR2HSV)
        # Equalize on S
        imgh, imgs, imgv = cv2.split(imghsv)
        imghsv = cv2.merge([imgh, cv2.equalizeHist(imgs), imgv])

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3), (-1,-1))

        self.shape = imghsv.shape

        '''Returns coloured region, its contour, and axis-aligned bounding rect'''
        def findColouredRegionContours(imghsv, colour):
            minArea = self.params['contourMinArea']/1000.0 * cvimg.shape[0] * cvimg.shape[1]
            hueLo, hueHi = self.params[colour+'HueLow'], self.params[colour+'HueHigh']
            satLo, satHi = self.params[colour+'SatLow'], self.params[colour+'SatHigh']
            valLo, valHi = self.params[colour+'ValLow'], self.params[colour+'ValHigh']

            # Perform a sum if hueHi < hueLo
            addRegions = False
            if hueHi < hueLo:
                addRegions = True
            if not addRegions:
                img = cv2.inRange(
                        imghsv,
                        np.array([hueLo, satLo, valLo],np.uint8),
                        np.array([hueHi, satHi, valHi],np.uint8)
                )
            else:
                img = cv2.inRange(
                        imghsv,
                        np.array([hueLo, satLo, valLo],np.uint8),
                        np.array([255, satHi, valHi],np.uint8)
                )
                img += cv2.inRange(
                        imghsv,
                        np.array([0, satLo, valLo],np.uint8),
                        np.array([hueHi, satHi, valHi],np.uint8)
                )

            # Close up gaps
            structuringElt = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3), (1,1))
            img = cv2.dilate(img, structuringElt)
            img = cv2.erode(img, structuringElt)

            tmp = img.copy() # findContours modifies the original
            contours, _ = cv2.findContours(tmp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            contours = filter(lambda c: cv2.contourArea(c) > minArea, contours)
            maxcontour = max(contours, key=cv2.contourArea) if contours else None
            boundingRect = cv2.boundingRect(maxcontour) if maxcontour is not None else (0,0,1,1)

            return (img, maxcontour, boundingRect)

        images = []
        choices = COLOURS if self.target == 'all' else [self.target]
        for colour in choices:
            images.append(findColouredRegionContours(imghsv, colour))

        self.regionCount = sum([contour is not None for _, contour, _ in images])

        imgCombinedBW = reduce(lambda x, y: x | y, [region[0] for region in images])

        minXY, maxXY = (99999,99999), (-1,-1)
        for image in images:
            x,y,w,h = image[2]
            if image[1] is not None:
                minXY = (min(minXY[0],x), min(minXY[1],y))
                maxXY = (max(maxXY[0],x+w), max(maxXY[1],y+h))
        self.bigBoundingRect = (minXY[0], minXY[1], maxXY[0]-minXY[0], maxXY[1]-minXY[1])

        imgMasked = np.zeros_like(imgCombinedBW, dtype=np.uint8)
        roicontours = None
        if self.regionCount:
            x,y,w,h = self.bigBoundingRect
            points = np.array([(x,y),(x,y+h),(x+w,y+h),(x+w,y)], dtype=np.int32)
            cv2.fillPoly(imgMasked, [points], 255)
            imgMasked = np.bitwise_and(imgCombinedBW, imgMasked)

            imgMasked = cv2.morphologyEx(imgMasked, cv2.MORPH_CLOSE, kernel, None, (-1,-1), 2)
            tmp = imgMasked.copy()
            roicontours, _ = cv2.findContours(tmp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            bigContour = max(roicontours, key=cv2.contourArea)

            # Find the quad covering the big contour:
            # Compute both MinAreaRect and ConvexHull.
            # Then, for each of the four points found by MinAreaRect,
            # find the corresponding nearest point in the convex hull.
            convexHull = cv2.convexHull(bigContour)
            minRectPts = cv2.cv.BoxPoints(cv2.minAreaRect(bigContour))
            self.bigQuad = [
                min(convexHull, key=lambda p: (qx-p[0][0])**2 + (qy-p[0][1])**2)[0]
                for qx,qy in minRectPts
            ]
            # Reorder the points such that the top-left is always the first
            # point (assuming that the points are already in clockwise order)
            ysorted = sorted(self.bigQuad, key=lambda p: p[1])
            xsorted = (sorted(ysorted[0:2], key=lambda p: p[0]), sorted(ysorted[2:], key=lambda p: p[0]))
            self.bigQuad = [xsorted[0][0], xsorted[0][1], xsorted[1][1], xsorted[1][0]]


        regions = [img[y:y+h, x:x+w] for (img, _, (x,y,w,h)) in images]

        # Code to look for the holes
        holes = []
        for i, region in enumerate(regions):
            hole = cv2.morphologyEx(region, cv2.MORPH_CLOSE, kernel)

            tmp = hole.copy()
            contours, hierarchy = cv2.findContours(tmp, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE, offset=images[i][2][0:2])

            # Contours of holes are those which have a parent
            holecontours = [contours[k] for k in filter(lambda k: hierarchy[0][k][3] > -1, range(len(contours)))]
            # Retrieve the centre of the 2nd-largest hole (if any)
            if len(self.history) <= i or self.history[i] is None:
                targetHoles = sorted(holecontours, key=cv2.contourArea)[-2:]
                targetContour = None if len(targetHoles)==0 else targetHoles[0]
                targetCentre = None
                targetArea = 0
                if targetContour is not None:
                    targetCentre = calcCentroid(targetContour)
                    #targetArea = cv2.contourArea(targetContour)
                if len(self.history) <= i:
                    self.history.append(targetCentre)
                else:
                    self.history[i] = targetCentre
            else:
                # Minimize distance
                prevPt = self.history[i]
                if not holecontours:
                    targetCentre = None
                else:
                    targetCentres = [calcCentroid(c) for c in holecontours]
                    targetCentre = min(targetCentres, key=lambda c: (prevPt[0]-c[0])**2 + (prevPt[1]-c[1])**2)
                    self.history[i] = targetCentre

            holes.append(targetCentre)

        if self.DEBUG:
            colours = [(255,0,0), (0,255,0), (0,0,255), (0,255,255)]
            imgDebug = cv2.merge([imgMasked]*3)
            if roicontours is not None:
                for i in range(-1,len(self.bigQuad)-1):
                    pt1 = (self.bigQuad[i][0], self.bigQuad[i][1])
                    pt2 = (self.bigQuad[i+1][0], self.bigQuad[i+1][1])
                    cv2.line(imgDebug, pt1, pt2, colours[i+1], 2)

            for target in holes:
                if target is not None:
                    pt = (int(target[0]), int(target[1]))
                    cv2.circle(imgDebug, pt, 2, (255,0,0), 2)

            self.camdebug.publishImage('bw', imgDebug)
            self.camdebug.publishImage('hsv', imghsv)


    def changeTarget(self, target):
        self.target = target
        self.history = []
        self.regionCount = 0


    # Callback for subscribing to compass data
    def gotHeading(self, msg):
        self.heading = msg.yaw

