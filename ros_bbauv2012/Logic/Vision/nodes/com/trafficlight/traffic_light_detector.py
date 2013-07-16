#!/usr/bin/env python2
'''
Code to identify RoboSub traffic lights
'''

import roslib; roslib.load_manifest('Vision')
import rospy
from sensor_msgs.msg import Image
from bbauv_msgs.msg import compass_data
from cv_bridge import CvBridge, CvBridgeError

import random
import numpy as np
import cv2
from collections import deque

#from com.histogram.histogram import bbHistogram

class TrafficLight:
    # Convert a ROS Image to the Numpy matrix used by cv2 functions
    def rosimg2cv(self, ros_image):
        # Convert from ROS Image to old OpenCV image
        frame = self.cvbridge.imgmsg_to_cv(ros_image, ros_image.encoding)
        # Convert from old OpenCV image to Numpy matrix
        return np.array(frame, dtype=np.uint8) #TODO: find out actual dtype

    def __init__(self, params, lock, camdebug=None):
        self.cvbridge = CvBridge()
        self.params = params
        self.lock = lock
        self.camdebug = camdebug
        self.DEBUG = camdebug is not None and camdebug.debugOn

        self.shape = None

        self.buoyDetected = False
        self.redCentre, self.redRadius = (-1,-1), 0

        self.history = deque(maxlen=5)

    # Callback for subscribing to Image topic
    def gotRosFrame(self, rosImage):
        cvimg = self.rosimg2cv(rosImage)
        self.gotFrame(cvimg)

    # Function that gets called after conversion from ROS Image to OpenCV image
    def gotFrame(self, cvimg):
        self.lock.acquire()

        self.shape = cvimg.shape
        imghsv = cv2.cvtColor(cvimg, cv2.cv.CV_BGR2HSV)

        openingSize = 1
        openingElt = cv2.getStructuringElement(cv2.MORPH_RECT, (2*openingSize+1, 2*openingSize+1))
        closingSize = 5
        closingElt = cv2.getStructuringElement(cv2.MORPH_RECT, (2*openingSize+1, 2*openingSize+1))
        termsLow = ['HueLow', 'SatLow', 'ValLow']
        termsHigh = ['HueHigh', 'SatHigh', 'ValHigh']
        def threshold(img, color):
            colorMin = np.array([self.params[color+s] for s in termsLow], np.uint8)
            colorMax = np.array([self.params[color+s] for s in termsHigh], np.uint8)

            if colorMin[0] > colorMax[0]:
                tmpMax = np.array([255, colorMax[1], colorMax[2]], np.uint8)
                tmpMin = np.array([0, colorMin[1], colorMin[2]], np.uint8)
                output = cv2.inRange(img, colorMin, tmpMax)
                output |= cv2.inRange(img, tmpMin, colorMax)
            else:
                output = cv2.inRange(img, colorMin, colorMax)

            output = cv2.morphologyEx(output, cv2.MORPH_OPEN, openingElt)
            output = cv2.morphologyEx(output, cv2.MORPH_CLOSE, closingElt)
            return output

        redImg = threshold(imghsv, 'red')
        yellowImg = threshold(imghsv, 'yellow')
        greenImg = threshold(imghsv, 'green')

        #TODO: determine where the red (circular) buoy is (relative to the others)
        tmp = redImg.copy()
        redContours, _ = cv2.findContours(tmp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        redCircles = [(cv2.minEnclosingCircle(c), cv2.contourArea(c)) for c in redContours]
        redCircles = [((c,r),a) for ((c,r),a) in redCircles if r > 5]
        if redCircles:
            if not self.history:
                sortedCircles = sorted(redCircles, key=lambda ((c,r),a): r)
                (curCentre, curRadius), curArea = sortedCircles[0]
                curAreaRatio = float(curArea)/(curRadius * curRadius)
                for (c, r), a in sortedCircles[1:]:
                    if abs(r - self.redRadius) < 5 and float(a)/(r*r) > curAreaRatio:
                        curCentre, curRadius = c, r
                        curAreaRatio = float(curArea)/(curRadius * curRadius)
                self.redCentre, self.redRadius = curCentre, curRadius
            else:
                # Minimize distance and radius difference
                prevCentre, prevRadius = self.history[0]
                bestCircle = min(redCircles, key=lambda ((c,r),a): (prevCentre[0]-c[0])**2 + (prevCentre[1]-c[1])**2 + (prevRadius-r)**2)[0]
                self.redCentre, self.redRadius = bestCircle

            self.history.appendleft((self.redCentre, self.redRadius))

            self.buoyDetected = True

        #TODO: identify buoys

        #TODO: find 3 largest contours
        imgCombined = redImg | yellowImg | greenImg
        tmp = imgCombined.copy()
        contours, _ = cv2.findContours(tmp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        minArea = self.params['contourMinArea']
        contourAreas = [(contour, cv2.contourArea(contour)) for contour in contours]
        contourAreas = filter(lambda (c,a): a >= minArea, contourAreas)
        contourAreas.sort(key=lambda x: x[1])
        contourAreas.reverse()
        contourAreas = contourAreas[0:3]

        #TODO: calculate distances apart

        #TODO: build a big bounding box around all and use that for centering

        # Display debug stream
        if self.DEBUG:
            imgDebug = cv2.merge([redImg]*3)
            if self.buoyDetected:
                ctr = (int(self.redCentre[0]), int(self.redCentre[1]))
                cv2.circle(imgDebug, ctr, 1, (0,0,255), 1)
                cv2.circle(imgDebug, ctr, int(self.redRadius), (0,0,255), 1)
            self.camdebug.publishImage('image_filter', imgDebug)

        self.lock.release()



# For testing
if __name__ == '__main__':
    rospy.init_node('trafficlight', anonymous=False)
    tmpparams = { 'contourMinArea': 0,
               'redHueLow': 0, 'redHueHigh': 0, 'redSatLow': 0, 'redSatHigh': 0, 'redValLow': 0, 'redValHigh': 0,
               'yellowHueLow': 0, 'yellowHueHigh': 0, 'yellowSatLow': 0, 'yellowSatHigh': 0, 'yellowValLow': 0, 'yellowValHigh': 0,
               'greenHueLow': 0, 'greenHueHigh': 0, 'greenSatLow': 0, 'greenSatHigh': 0, 'greenValLow': 0, 'greenValHigh': 0
    }
    import threading
    trafficlight = TrafficLight(tmpparams, threading.Lock())

    # Set up param configuration window
    from dynamic_reconfigure.server import Server
    from Vision.cfg import TrafficLightConfig
    def configCallback(config, level):
        for param in tmpparams:
            tmpparams[param] = config[param]
        return config
    srv = Server(TrafficLightConfig, configCallback)

    rospy.Subscriber('/stereo_camera/left/image_rect_color', Image, trafficlight.gotRosFrame)

    rospy.spin()
