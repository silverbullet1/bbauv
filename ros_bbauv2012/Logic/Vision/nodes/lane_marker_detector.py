#!/usr/bin/env python2

# Code to identify RoboSub lane markers

import roslib; roslib.load_manifest('Vision')
import rospy
from sensor_msgs.msg import Image

from bbauv_msgs.msg import compass_data

import math
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError


# Helper function to normalize heading
def normHeading(heading):
    if heading > 360:
        return heading - 360
    if heading < 0:
        return heading + 360
    return heading

# Helper function to draw a histogram image
def get_hist_img(cv_img):
    hist_bins = 256
    hist_ranges = [(0,255)]

    hist, _ = np.histogram(cv_img, hist_bins, (0, 255))
    maxVal = np.max(hist)

    histImg = np.array( [255] * (hist_bins * hist_bins), dtype=np.uint8 ).reshape([hist_bins, hist_bins])
    hpt = int(0.9 * hist_bins)

    for h in range(hist_bins):
        binVal = float(hist[h])
        intensity = int(binVal * hpt / maxVal)
        cv2.line(histImg, (h, hist_bins), (h, hist_bins-intensity), 0)

    return histImg


# Actual lane marker detection class
class LaneDetector:
    # Convert a ROS Image to the Numpy matrix used by cv2 functions
    def rosimg2cv(self, ros_image):
        # Convert from ROS Image to old OpenCV image
        frame = self.cvbridge.imgmsg_to_cv(ros_image, ros_image.encoding)
        # Convert from old OpenCV image to Numpy matrix
        return np.array(frame, dtype=np.uint8) #TODO: find out actual dtype


    def __init__(self):
        self.cvbridge = CvBridge()

        # Initial state
        self.heading = 0.0

        # Configurable parameters
        self.params = { 'hueLow': 11, 'hueHigh': 30, 'contourMinArea': 300 }

        # Set up param configuration window
        def paramSetter(key):
            def setter(val):
                self.params[key] = val
            return setter
        cv2.namedWindow("settings", cv2.CV_WINDOW_AUTOSIZE)
        cv2.createTrackbar("Hue Low:", "settings", self.params['hueLow'], 180, paramSetter('hueLow'));
        cv2.createTrackbar("Hue High:", "settings", self.params['hueHigh'], 180, paramSetter('hueHigh'));
        cv2.createTrackbar("Min contour area:", "settings", self.params['contourMinArea'], 3000, paramSetter('contourMinArea'));

        ## Example filters
        ## Original image - its Laplacian
        #tmpkernel = np.array([[0., -1., 0.],[-1., 5., -1.], [0., -1., 0.]])
        #sublaplacefilter = lambda (cv_img): cv2.filter2D(cv_img, -1, tmpkernel)

        ## Canny (edge detection)
        #cannyfilter = lambda (_): cv2.threshold(cv2.Canny(grayfilter.image, 125, 350), 128, 255, cv2.THRESH_BINARY_INV)[1]


    # Callback for subscribing to Image topic
    def gotRosFrame(self, rosImage):
        cvimg = self.rosimg2cv(rosImage)
        self.gotFrame(cvimg)

    # Function that gets called after conversion from ROS Image to OpenCV image
    def gotFrame(self, cvimg):
        # Perform some processing

        # Ignore all blue stuff
#        channels = cv2.split(cvimg)
#        imgNew = cv2.merge([np.zeros(channels[1].shape, channels[1].dtype), channels[1], channels[2]])

        # Find the orange regions
        imghsv = cv2.cvtColor(cvimg, cv2.cv.CV_BGR2HSV)
#        imghsv = cv2.equalizeHist(imghsv)

        imgBW = cv2.inRange(imghsv, np.array([self.params['hueLow'],0,0]), np.array([self.params['hueHigh'], 255, 255]))

        structuringElt = cv2.getStructuringElement(cv2.MORPH_RECT,
                                (3,3), (1,1))
        imgBW = cv2.dilate(imgBW, structuringElt)
        imgBW = cv2.erode(imgBW, structuringElt)

        # Make a copy because findContours modifies the original image
        imgBW2 = imgBW.copy()

        # Retrieve the contours of the black regions
        contours, _ = cv2.findContours(imgBW2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # Find the bounding rectangles of the contours that are larger than our threshold
        # Give an array of rects in tuples of the form: ( (center_x,center_y), (w,h), theta )
        foundRects = [ ]
        for contour in contours:
            curArea = cv2.contourArea(contour)
            if curArea >= self.params['contourMinArea']:
                maxRect = cv2.minAreaRect(contour)
                foundRects.append(maxRect)


        # Perform operations on rectangles found
        for rect in foundRects:
            # Obtain the actual corners of the box
            points = cv2.cv.BoxPoints(rect)
            # Draw the lines
            for i in range(4):
                # The line function doesn't accept floating point values
                pt1 = (int(points[i][0]), int(points[i][1]))
                pt2 = (int(points[(i+1)%4][0]), int(points[(i+1)%4][1]))
                cv2.line(imgBW, pt1, pt2, 255, 1)

            #TODO: publish the rects (with target headings)

#            # Prepare other data about the bounding rect for the state machine
#            rectData['points'] = points
#            rectData['edges'] = edges = [ np.array(points[1]) - np.array(points[0]), np.array(points[2]) - np.array(points[1]) ]
#            # Calculate angle from the vertical
#            if cv2.norm(edges[0]) > cv2.norm(edges[1]):
#                rectData['angle'] = math.atan(edges[0][0] / edges[0][1]) / math.pi * 180
#            else:
#                rectData['angle'] = math.atan(edges[1][0] / edges[1][1]) / math.pi * 180
#
#            if rectData['angle'] == 90:
#                if maxRect[0][0] > (cvimg.shape[1] / 2):
#                    rectData['angle'] = -90

        # Screens for debugging
        cv2.imshow("src", cvimg)
        cv2.imshow("gray", imghsv)
        cv2.imshow("bw", imgBW)


    # Callback for subscribing to compass data
    def gotHeading(self, msg):
        self.heading = msg.yaw


# Main
if __name__ == '__main__':
    rospy.init_node('line_follower', anonymous=True)
    loopRateHz = rospy.get_param('~loopHz', 20)
    imageTopic = rospy.get_param('~image', '/bottomcam/camera/image_raw')
    compassTopic = rospy.get_param('~compass', '/os5000_data')

    app = LaneDetector()

    rospy.Subscriber(imageTopic, Image, app.gotRosFrame)
#    rospy.Subscriber(compassTopic, compass_data, app.gotHeading)
#    movementPub = rospy.Publisher('/line_follower', controller_input)

    r = rospy.Rate(loopRateHz)
    while not rospy.is_shutdown():
        key = cv2.waitKey(20)
        if key == 27: # Exit on getting the Esc key
            break

# vim: set sw=4 ts=4 expandtab:
