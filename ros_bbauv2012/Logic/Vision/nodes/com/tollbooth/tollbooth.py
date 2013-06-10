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

        # Initial state
        self.heading = 0.0
        self.regionCount = 0


    # Callback for subscribing to Image topic
    def gotRosFrame(self, rosImage):
        cvimg = self.rosimg2cv(rosImage)
        self.gotFrame(cvimg)

    # Function that gets called after conversion from ROS Image to OpenCV image
    def gotFrame(self, cvimg):
        imghsv = cv2.cvtColor(cvimg, cv2.cv.CV_BGR2HSV)

        '''Returns coloured region, its contour, and axis-aligned bounding rect'''
        def findColouredRegionContours(imghsv, colour, prevContours=None):
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

            if prevContours is not None:
                prevContours.append(contours or [])

            return (img, maxcontour, boundingRect)

        images, foundContours = [], []
        for colour in ['red', 'blue', 'yellow', 'green']:
            images.append(findColouredRegionContours(imghsv, colour, foundContours))

        self.regionCount = sum([contour is not None for _, contour, _ in images])

        imgCombinedBW = reduce(lambda x, y: x | y, [region[0] for region in images])

        minXY, maxXY = (99999,99999), (-1,-1)
        for image in images:
            x,y,w,h = image[2]
            if image[1] is not None:
                minXY = (min(minXY[0],x), min(minXY[1],y))
                maxXY = (max(maxXY[0],x+w), max(maxXY[1],y+h))
        self.bigBoundingRect = (minXY[0], minXY[1], maxXY[0]-minXY[0], maxXY[1]-minXY[1])

        mask = np.zeros_like(imgCombinedBW, dtype=np.uint8)
        #points = cv2.cv.BoxPoints(self.bigBoundingRect)
        #cv2.fillPoly(mask, [np.int32(points)], 255)
        #imgMasked = np.bitwise_and(imgCombinedBW, mask)

        #tmp = imgCombinedBW.copy()
        #contours, _ = cv2.findContours(tmp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)


        regions = [img[y:y+h, x:x+w] for (img, _, (x,y,w,h)) in images]

        # Code to look for the holes
        holes = []
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3), (-1,-1))
        for region in regions:
            hole = cv2.morphologyEx(region, cv2.MORPH_CLOSE, kernel)
            holes.append(cv2.morphologyEx(hole, cv2.MORPH_GRADIENT, kernel))

        if holes:
            hole = holes[0]
            tmp = hole.copy()
            contours, _ = cv2.findContours(tmp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            #TODO: filter out the small ones

        if self.DEBUG:
            imgDebug = cv2.merge([imgCombinedBW]*3)
            if images:
                x,y,w,h = images[0][2]
                cv2.rectangle(imgDebug, (x,y), (x+w,y+h), (255, 0, 0), 3)
                x,y,w,h = images[1][2]
                cv2.rectangle(imgDebug, (x,y), (x+w,y+h), (0, 0, 255), 3, 2)
                x,y,w,h = images[2][2]
                cv2.rectangle(imgDebug, (x,y), (x+w,y+h), (255, 255, 0), 3)
                x,y,w,h = images[3][2]
                cv2.rectangle(imgDebug, (x,y), (x+w,y+h), (0, 255, 0), 3)

                x,y,w,h = self.bigBoundingRect
                cv2.rectangle(imgDebug, (x,y), (x+w,y+h), (0, 255, 255), 4)

#            imgDebug = cv2.merge([hole]*3)
#            for contour in contours:
#                cv2.drawContours(imgDebug, [contour], 0, (random.randint(0,255), random.randint(0,255), random.randint(0,255)), 1)

            self.camdebug.publishImage('bw', imgDebug)
            self.camdebug.publishImage('hsv', imghsv)


#        # Retrieve the contours of the important regions
#        contours, _ = cv2.findContours(imgBW2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
#        # Find the bounding rects of the contours that are larger than our threshold
#        areaThreshold = params['contourMinArea'] * cvimg.shape[0] * cvimg.shape[1] / 1000
#        contourRects = [cv2.minAreaRect(contour) for contour in
#            filter(
#                lambda contour: cv2.contourArea(contour) >= areaThreshold,
#                contours)
#        ]
#
#        debugTmp, debugTmp2 = None, None
#        if self.DEBUG:
#            debugTmp = np.zeros_like(imgBW)
#            debugTmp2 = cv2.merge([debugTmp, debugTmp, debugTmp])
#
#        foundLines = []
#
#        # Find lines in each region
#        for rect in contourRects:
#            mask = np.zeros_like(imgBW, dtype=np.uint8)
#            points = np.int32(cv2.cv.BoxPoints(rect))
#            cv2.fillPoly(mask, [points], 255)
#            rectImg = np.bitwise_and(imgBW, mask)
#
#            if self.DEBUG:
#                debugTmp = np.bitwise_or(rectImg, debugTmp)
#
#            lines = cv2.HoughLinesP(rectImg, 1, cv2.cv.CV_PI/180, 80, None, 30, 10)
#
#            # Find median gradient of lines
#            # Store as (x,y) (centre of box), angle (orientation of box)
#            gradient = 0
#            if lines != None:
#                gradients = [math.atan2(endy-starty, endx-startx)
#                                for startx, starty, endx, endy in lines[0]]
#                gradient = np.median(gradients)
#
#                angle = np.rad2deg(gradient)
#
#                if angle == 0:
#                    # Need some way to distinguish between left-pointing and right-pointing
#                    # when the line is horizontal:
#                    # we'll assume that if the centre of the box is on the right, it points to the right
#                    if rect[0][0] > (rectImg.shape[1] / 2):
#                        angle = 180
#
#                # Try to correct the angles if we have more lines to test against
#                if len(foundLines) > 0:
#                    # Construct rays from the centres of the boxes
#                    ray1 = get_ray(foundLines[0]['pos'], foundLines[0]['angle'])
#                    ray2 = get_ray(rect[0], angle)
#                    (t1, t2) = ray_intersection(ray1, ray2)
#
#                    if t2 > 0:
#                        angle += (180 if angle < 0 else -180)
#                    if t1 > 0:
#                        foundLines[0]['angle'] += (180 if foundLines[0]['angle'] < 0 else -180)
#
#                foundLines.append({'pos':rect[0], 'angle':angle})
#
#                if self.DEBUG:
#                    # Draw bounding rect
#                    for i in range(4):
#                        # The line function doesn't accept floating point values
#                        pt1 = (points[i][0], points[i][1])
#                        pt2 = (points[(i+1)%4][0], points[(i+1)%4][1])
#                        cv2.line(debugTmp2, pt1, pt2, (0,0,255), 1)
#
#        for line in foundLines:
#            line['heading'] = norm_heading(self.heading + norm_heading(-line['angle'] - 90))
#
#        self.foundLines = foundLines
#        if self.frameCallback:
#            self.frameCallback()
#
#        # Screens for debugging
#        if self.DEBUG:
#            for i,line in enumerate(foundLines):
#                # Draw a debug line
#                startpt = line['pos']
#                gradient = np.deg2rad(line['angle'])
#                endpt = (int(startpt[0] + 100 * math.cos(gradient)), int(startpt[1] + 100 * math.sin(gradient)))
#                startpt = (int(startpt[0]), int(startpt[1]))
#                cv2.line(debugTmp2, startpt, endpt, (255,0,0), 2)
#
#            imgDebug = np.bitwise_xor(cv2.merge([debugTmp, debugTmp, debugTmp]), debugTmp2)
#            self.camdebug.publishImage('hsv', imghsv)
#            self.camdebug.publishImage('bw', imgDebug)


    # Callback for subscribing to compass data
    def gotHeading(self, msg):
        self.heading = msg.yaw

