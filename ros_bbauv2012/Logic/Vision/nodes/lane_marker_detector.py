#!/usr/bin/env python2
'''
Code to identify RoboSub lane markers
'''

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

# Helper function to parameterize a ray
def get_ray(pt, angle):
    radians = angle / 180.0 * math.pi
    u = -math.sin(radians)
    v = math.cos(radians)
    return ( pt, (u,v) )

# Helper function to get the t's of the intersection of the parameterized rays
def ray_intersection(ray1, ray2):
    ((x1, y1), (u1, v1)) = ray1
    ((x2, y2), (u2, v2)) = ray2
    det = 1.0 / (u2*v1 - u1*v2)
    xd, yd = x2 - x1, y2 - y1
    t1 = det * (-v2 * xd + u2 * yd)
    t2 = det * (-v1 * xd + u1 * yd)
    return (t1, t2)

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
        self.params = { 'hueLow': 11, 'hueHigh': 35, 'contourMinArea': 15 }

        # Set up param configuration window
        def paramSetter(key):
            def setter(val):
                self.params[key] = val
            return setter
        cv2.namedWindow("settings", cv2.CV_WINDOW_AUTOSIZE)
        cv2.createTrackbar("Hue Low:", "settings", self.params['hueLow'], 180, paramSetter('hueLow'));
        cv2.createTrackbar("Hue High:", "settings", self.params['hueHigh'], 180, paramSetter('hueHigh'));
        cv2.createTrackbar("Min contour area (1/1000):", "settings", self.params['contourMinArea'], 1000, paramSetter('contourMinArea'));

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

        # Close up the gaps in the detected regions
        structuringElt = cv2.getStructuringElement(cv2.MORPH_RECT,
                                (3,3), (1,1))
        imgBW = cv2.dilate(imgBW, structuringElt)
        imgBW = cv2.erode(imgBW, structuringElt)

        # Make a copy because findContours modifies the original image
        imgBW2 = imgBW.copy()

        # Retrieve the contours of the black regions
        contours, _ = cv2.findContours(imgBW2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # Find the bounding rectangles of the contours that are larger than our threshold
        # Give an array of rects in dictionaries of the form:
        # {
        #    'rect':   ((center_x,center_y), (w,h), theta),
        #    'points': [... array of the 4 corners of the rect ...],
        #    'edges':  [... array of 2 perpendicular edges (as vectors) of the rect ...],
        #    'angle':  (float) angle of the box from the vertical (0 deg), with positive counterclockwise,
        #    'heading':(float) heading that the box should be pointing to
        # }
        foundRects = [ ]
        for contour in contours:
            curArea = cv2.contourArea(contour)
            if curArea >= self.params['contourMinArea'] * cvimg.shape[0] * cvimg.shape[1] / 1000:
                maxRect = cv2.minAreaRect(contour)
                foundRects.append({'rect': maxRect})

        # Perform operations on rectangles found
        count = 0
        for rect in foundRects:
            # Obtain the actual corners of the box
            points = cv2.cv.BoxPoints(rect['rect'])

            rect['points'] = points
            rect['edges'] = edges = [ np.array(points[1]) - np.array(points[0]), np.array(points[2]) - np.array(points[1]) ]

            # Calculate angle from the vertical
            if cv2.norm(edges[0]) > cv2.norm(edges[1]):
                rect['angle'] = math.atan(edges[0][0] / edges[0][1]) / math.pi * 180
            else:
                rect['angle'] = math.atan(edges[1][0] / edges[1][1]) / math.pi * 180

            if rect['angle'] == 90:
                # Need some way to distinguish between left-pointing and right-pointing
                # when the box is horizontal:
                # we'll assume that if the centre of the box is on the right, it points to the right
                if rect['rect'][0][0] > (cvimg.shape[1] / 2):
                    rect['angle'] = -90

            # Try to correct the angles if we have more lines to test against
            if count > 0:
                # Construct rays from the centres of the boxes
                ray1 = get_ray(foundRects[0]['rect'][0], foundRects[0]['angle'])
                ray2 = get_ray(rect['rect'][0], rect['angle'])
                (t1, t2) = ray_intersection(ray1, ray2)
                # We want t1 and t2 to both be negative
                # If t1 is positive, flip ray2; and vice versa
                if t1 > 0:
                    rect['angle'] += 180 if rect['angle'] < 0 else -180
                if t2 > 0:
                    foundRects[0]['angle'] += 180 if foundRects[0]['angle'] < 0 else -180

            count += 1

            # Draw the lines for debugging
            imgBW = cv2.merge([imgBW, imgBW, imgBW])
            for i in range(4):
                # The line function doesn't accept floating point values
                pt1 = (int(points[i][0]), int(points[i][1]))
                pt2 = (int(points[(i+1)%4][0]), int(points[(i+1)%4][1]))
                cv2.line(imgBW, pt1, pt2, (255,255,0), 2)

        #TODO: publish the rects (with target headings)

        # Debug output
        count = 0
        for rect in foundRects:
            print 'Angle', count, ':', rect['angle']
            count += 1

        # Screens for debugging
#        cv2.imshow("src", cvimg)
        cv2.imshow("hsv", imghsv)
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

        r.sleep()

# vim: set sw=4 ts=4 expandtab:
