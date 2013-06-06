#!/usr/bin/env python2
'''
Class to identify RoboSub lane markers and provide headings
'''
import roslib; roslib.load_manifest('Vision')
from sensor_msgs.msg import Image
from bbauv_msgs.msg import compass_data, depth
from cv_bridge import CvBridge, CvBridgeError

import math
import numpy as np
import cv2

from com.utils.utils import norm_heading, to_heading_space

# Helper function to parameterize a ray
def get_ray(pt, angle):
    radians = angle / 180.0 * math.pi
    u = math.cos(radians)
    v = math.sin(radians)
    return (pt, (u,v))

# Helper function to get the t's of the intersection of the parameterized rays
def ray_intersection(ray1, ray2):
    ((x1, y1), (u1, v1)) = ray1
    ((x2, y2), (u2, v2)) = ray2
    det = 1.0 / (u2*v1 - u1*v2)
    xd, yd = x2 - x1, y2 - y1
    t1 = det * (-v2 * xd + u2 * yd)
    t2 = det * (-v1 * xd + u1 * yd)
    return (t1, t2)


class LaneDetector:
    # Convert a ROS Image to the Numpy matrix used by cv2 functions
    def rosimg2cv(self, ros_image):
        # Convert from ROS Image to old OpenCV image
        frame = self.cvbridge.imgmsg_to_cv(ros_image, ros_image.encoding)
        # Convert from old OpenCV image to Numpy matrix
        return np.array(frame, dtype=np.uint8) #TODO: find out actual dtype

    '''
    params: dynamic_reconfigure params; see LaneMarkerDetector.cfg
    DEBUG:  turns on/off DEBUG mode output streams
    '''
    def __init__(self, params, camdebug=None):
        self.cvbridge = CvBridge()
        self.foundLines = []
        self.frameCallback = None
        self.inputHeading = 0

        self.params = params
        self.camdebug = camdebug
	self.DEBUG = camdebug is not None and camdebug.debugOn

        # Initial state
        self.heading = 0.0
        self.depth = 0.0
        self.offset = (0,0)


    # Callback for subscribing to Image topic
    def gotRosFrame(self, rosImage):
        cvimg = self.rosimg2cv(rosImage)
        self.gotFrame(cvimg)

    # Function that gets called after conversion from ROS Image to OpenCV image
    def gotFrame(self, cvimg):
        filteredImg = cv2.GaussianBlur(cvimg, (3,3), 0)

        # Find the orange regions
        imghsv = cv2.cvtColor(cvimg, cv2.cv.CV_BGR2HSV)
        imghue, imgsat, imgval = cv2.split(imghsv)
        imghsv = cv2.merge([imghue, cv2.equalizeHist(imgsat), imgval])

        imgBW = np.array(cv2.inRange(imghsv,
                    np.array([self.params['hueLow'], self.params['satLow'], self.params['valLow']], np.uint8),
                    np.array([self.params['hueHigh'], self.params['satHigh'], self.params['valHigh']], np.uint8)))
        imgBW = np.invert(imgBW)

        # Close up the gaps in the detected regions
        structuringElt = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3), (-1,-1))
        imgBW = cv2.dilate(imgBW, structuringElt)
        imgBW = cv2.erode(imgBW, structuringElt)

        # Make a copy because findContours modifies the original image
        imgBW2 = imgBW.copy()

        # Retrieve the contours of the important regions
        contours, _ = cv2.findContours(imgBW2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # Find the bounding rects of the contours that are larger than our threshold
        areaThreshold = self.params['contourMinArea'] * cvimg.shape[0] * cvimg.shape[1] / 1000
        contourRects = [cv2.minAreaRect(contour) for contour in
            filter(
                lambda contour: cv2.contourArea(contour) >= areaThreshold,
                contours)
        ]

        centroid = (-1,-1)
        self.offset = reduce(lambda a,b: (a[0]+b[0],a[1]+b[1]), [r[0] for r in contourRects], (0,0))
        if len(contourRects):
            centroid = self.offset = (self.offset[0]/len(contourRects), self.offset[1]/len(contourRects))
            shape = imgBW.shape
            self.offset = (self.offset[0]/float(shape[1])-0.5, self.offset[1]/float(shape[0])-0.5) 

        if self.DEBUG:
            debugTmp = np.zeros_like(imgBW)
            debugTmp2 = cv2.merge([debugTmp, debugTmp, debugTmp])
            debugThresh = imgBW.copy()

        foundLines = []

        # Find lines in each region
        for rect in contourRects:
            mask = np.zeros_like(imgBW, dtype=np.uint8)
            points = np.int32(cv2.cv.BoxPoints(rect))
            cv2.fillPoly(mask, [points], 255)
            rectImg = np.bitwise_and(imgBW, mask)

            if self.DEBUG:
                debugTmp = np.bitwise_or(rectImg, debugTmp)

            lines = cv2.HoughLinesP(rectImg, 1, cv2.cv.CV_PI/180, 80, None, 60, 10)

            # Find median gradient of lines
            # Store as (x,y) (centre of box), angle (orientation of box)
            gradient = 0
            if lines != None:
                gradients = [math.atan2(endy-starty, endx-startx)
                                for startx, starty, endx, endy in lines[0]]
                gradient = np.median(gradients)

                angle = np.rad2deg(gradient)

                # Correct angle based on the input heading (always go away from the last task)
                if 90 < abs(self.inputHeading - norm_heading(self.heading+to_heading_space(angle))) < 270:
                    angle += (180 if angle < 0 else - 180)

                # Try to correct the angles if we have more lines to test against
                if len(foundLines) > 0:
                    if abs(foundLines[0]['angle'] - angle) < 0.0005 or abs(180 - abs(foundLines[0]['angle'] - angle)) < 0.0005:
                        angle = foundLines[0]['angle']
                    else:
                        # Construct rays from the centres of the boxes
                        ray1 = get_ray(foundLines[0]['pos'], foundLines[0]['angle'])
                        ray2 = get_ray(rect[0], angle)
                        (t1, t2) = ray_intersection(ray1, ray2)

                        if t2 > 0:
                            angle += (180 if angle < 0 else -180)
                        if t1 > 0:
                            foundLines[0]['angle'] += (180 if foundLines[0]['angle'] < 0 else -180)

                foundLines.append({'pos':rect[0], 'angle':angle})

                if self.DEBUG:
                    # Draw bounding rect
                    for i in range(4):
                        # The line function doesn't accept floating point values
                        pt1 = (points[i][0], points[i][1])
                        pt2 = (points[(i+1)%4][0], points[(i+1)%4][1])
                        cv2.line(debugTmp2, pt1, pt2, (0,0,255), 1)

        for line in foundLines:
            line['heading'] = norm_heading(self.heading + to_heading_space(line['angle']))

        self.foundLines = foundLines
        if self.frameCallback:
            self.frameCallback()

        # Screens for debugging
        if self.DEBUG:
            for i,line in enumerate(foundLines):
                # Draw a debug line
                startpt = line['pos']
                gradient = np.deg2rad(line['angle'])
                endpt = (int(startpt[0] + 100 * math.cos(gradient)), int(startpt[1] + 100 * math.sin(gradient)))
                startpt = (int(startpt[0]), int(startpt[1]))
                cv2.line(debugTmp2, startpt, endpt, (255,0,0), 2)

            imgDebug = np.bitwise_xor(cv2.merge([debugTmp, debugTmp, debugTmp]), debugTmp2)
            cv2.circle(imgDebug, (int(centroid[0]),int(centroid[1])), 2, (255,0,0), 2)
            self.camdebug.publishImage('hsv', imghsv)
            self.camdebug.publishImage('thresh', debugThresh)
            #self.camdebug.publishImage('bw', imgDebug)
            self.camdebug.publishImage('image_filter', imgDebug)

    # Other callbacks
    def gotHeading(self, msg):
        self.heading = msg.yaw
    def gotDepth(self, msg):
        self.depth = msg.depth

