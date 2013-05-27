#!/usr/bin/env python2
'''
Code to identify RoboSub lane markers
'''

import roslib; roslib.load_manifest('Vision')
import rospy
from sensor_msgs.msg import Image
from bbauv_msgs.msg import compass_data
from bbauv_msgs.msg import depth

import actionlib
import PID_Controller.msg
from PID_Controller.msg import ControllerAction

import math
from random import randint
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from com.camdebug.camdebug import CamDebug

from dynamic_reconfigure.server import Server
from Vision.cfg import LaneMarkerDetectorConfig

import smach


DEBUG = True
camdebug = None

params = { 'hueLow': 11, 'hueHigh': 65, 'contourMinArea': 15 }
laneDetector = None


# Helper function to normalize heading
def norm_heading(heading):
    if heading > 360:
        return heading - 360
    if heading < 0:
        return heading + 360
    return heading

# Helper function to parameterize a ray
def get_ray(pt, angle):
    radians = angle / 180.0 * math.pi
    u = math.cos(radians)
    v = math.sin(radians)
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
        self.foundLines = []
        self.frameCallback = None

        # Initial state
        self.heading = 0.0
        self.depth = 0.0


    # Callback for subscribing to Image topic
    def gotRosFrame(self, rosImage):
        cvimg = self.rosimg2cv(rosImage)
        self.gotFrame(cvimg)

    # Function that gets called after conversion from ROS Image to OpenCV image
    def gotFrame(self, cvimg):
        # Find the orange regions
        imghsv = cv2.cvtColor(cvimg, cv2.cv.CV_BGR2HSV)
#        imghsv = cv2.equalizeHist(imghsv)

        imgBW = np.array(cv2.inRange(imghsv, np.array([params['hueLow'],0,0],np.uint8), np.array([params['hueHigh'], 255, 255],np.uint8)))

        # Close up the gaps in the detected regions
        structuringElt = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3), (1,1))
        imgBW = cv2.dilate(imgBW, structuringElt)
        imgBW = cv2.erode(imgBW, structuringElt)

        # Make a copy because findContours modifies the original image
        imgBW2 = imgBW.copy()

        # Retrieve the contours of the important regions
        contours, _ = cv2.findContours(imgBW2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # Find the bounding rects of the contours that are larger than our threshold
        areaThreshold = params['contourMinArea'] * cvimg.shape[0] * cvimg.shape[1] / 1000
        contourRects = [cv2.minAreaRect(contour) for contour in
            filter(
                lambda contour: cv2.contourArea(contour) >= areaThreshold,
                contours)
        ]

        debugTmp, debugTmp2 = None, None
        if DEBUG:
            debugTmp = np.zeros_like(imgBW)
            debugTmp2 = cv2.merge([debugTmp, debugTmp, debugTmp])

        foundLines = []

        # Find lines in each region
        for rect in contourRects:
            mask = np.zeros_like(imgBW, dtype=np.uint8)
            points = np.int32(cv2.cv.BoxPoints(rect))
            cv2.fillPoly(mask, [points], 255)
            rectImg = np.bitwise_and(imgBW, mask)

            if DEBUG:
                debugTmp = np.bitwise_or(rectImg, debugTmp)

            lines = cv2.HoughLinesP(rectImg, 1, cv2.cv.CV_PI/180, 80, None, 30, 10)

            # Find median gradient of lines
            # Store as (x,y) (centre of box), angle (orientation of box)
            gradient = 0
            if lines != None:
                gradients = [math.atan2(endy-starty, endx-startx)
                                for startx, starty, endx, endy in lines[0]]
                gradient = np.median(gradients)

                angle = np.rad2deg(gradient)

                if angle == 0:
                    # Need some way to distinguish between left-pointing and right-pointing
                    # when the line is horizontal:
                    # we'll assume that if the centre of the box is on the right, it points to the right
                    if rect[0][0] > (rectImg.shape[1] / 2):
                        angle = 180

                # Try to correct the angles if we have more lines to test against
                if len(foundLines) > 0:
                    # Construct rays from the centres of the boxes
                    ray1 = get_ray(foundLines[0]['pos'], foundLines[0]['angle'])
                    ray2 = get_ray(rect[0], angle)
                    (t1, t2) = ray_intersection(ray1, ray2)

                    if t2 > 0:
                        angle += (180 if angle < 0 else -180)
                    if t1 > 0:
                        foundLines[0]['angle'] += (180 if foundLines[0]['angle'] < 0 else -180)

                foundLines.append({'pos':rect[0], 'angle':angle})

                if DEBUG:
                    # Draw bounding rect
                    for i in range(4):
                        # The line function doesn't accept floating point values
                        pt1 = (points[i][0], points[i][1])
                        pt2 = (points[(i+1)%4][0], points[(i+1)%4][1])
                        cv2.line(debugTmp2, pt1, pt2, (0,0,255), 1)

        for line in foundLines:
            line['heading'] = norm_heading(self.heading + norm_heading(-line['angle'] - 90))

        self.foundLines = foundLines
        if self.frameCallback:
            self.frameCallback()

        # Screens for debugging
        if DEBUG:
            for i,line in enumerate(foundLines):
                # Draw a debug line
                startpt = line['pos']
                gradient = np.deg2rad(line['angle'])
                endpt = (int(startpt[0] + 100 * math.cos(gradient)), int(startpt[1] + 100 * math.sin(gradient)))
                startpt = (int(startpt[0]), int(startpt[1]))
                cv2.line(debugTmp2, startpt, endpt, (255,0,0), 2)

            imgDebug = np.bitwise_xor(cv2.merge([debugTmp, debugTmp, debugTmp]), debugTmp2)
            camdebug.publishImage('hsv', imghsv)
            camdebug.publishImage('bw', imgDebug)


    # Callback for subscribing to compass data
    def gotHeading(self, msg):
        self.heading = msg.yaw
    def gotDepth(self, msg):
        self.depth = msg.depth


class MedianFilter:
    def __init__(self, sampleWindow=30):
        self.samples = []
        self.sampleWindow = sampleWindow
        self.lastSampled = rospy.Time()

    def newSample(self, sample):
        curTime = rospy.Time()
        # Discard previous samples if we only sampled them a long time ago
        if (curTime - self.lastSampled) > rospy.Duration(1,0):
            self.samples = []

        self.lastSampled = curTime
        if len(self.samples) >= self.sampleWindow:
            self.samples.pop(0)
        self.samples.append(sample)

        self.median = np.median(self.samples)
        return self.median

    def getVariance(self):
        if len(self.samples) >= self.sampleWindow:
            return np.var(self.samples)
        else:
            return 999 # Just a big value



'''
States
'''
class SearchState(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['foundLane', 'aborted'],
                             input_keys=['expectedLanes'],
                             output_keys=['expectedLanes'])

    def execute(self, userdata):
        global laneDetector
        laneDetector = LaneDetector()

        actionClient = actionlib.SimpleActionClient('search', ControllerAction)
        actionClient.wait_for_server()

        #TODO: Move around in a smarter way to keep all lanes on screen
        rospy.loginfo('moving around to find lane')
        while len(laneDetector.foundLines) != userdata.expectedLanes:
            if rospy.is_shutdown(): return 'aborted'

            #TODO: use values related to spin rate
            goal = PID_Controller.msg.ControllerGoal()
            goal.heading_setpoint = self.heading + 0.1
            goal.depth_setpoint = self.depth
            goal.forward_setpoint = 0.1

            actionClient.sendGoal(goal)
            actionClient.waitForResult() #TODO: use a timeout here?

            rosRate.sleep()

        goal = PID_Controller.msg.ControllerGoal()
        goal.heading_setpoint = self.heading
        goal.depth_setpoint = self.depth
        actionClient.sendGoal(goal) # Don't wait, just continue

        return 'foundLane'

class ConfirmingState(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['confirmed', 'searchAgain', 'aborted'],
                             input_keys=['expectedLanes'],
                             output_keys=['expectedLanes', 'headings'])

    def execute(self, userdata):
        self.headings = []
        self.medianFilters = []
        for i in range(userdata.expectedLanes):
            self.medianFilters.append(MedianFilter())

        global laneDetector
        laneDetector = LaneDetector()

        self.status = 'started'

        # Check the last few headings and see if they lie within a certain
        # tolerance level
        rospy.loginfo('checking last few headings')
        def frameCallback():
            if len(laneDetector.foundLines) >= userdata.expectedLanes:
                headings = sorted([line['heading'] for line in laneDetector.foundLines[0:userdata.expectedLanes]])
                for i, heading in enumerate(headings):
                    self.medianFilters[i].newSample(heading)
                self.status = 'searching'
            else:
                self.status = 'lost'

        laneDetector.frameCallback = frameCallback

        while True:
            if rospy.is_shutdown(): return 'aborted'
            if self.status == 'lost': return 'searchAgain'
            if all([medianFilter.getVariance() < 5 for medianFilter in self.medianFilters]):
                userdata.headings = [m.median for m in self.medianFilters]
                return 'confirmed'

            rosRate.sleep()

        return 'searchAgain'

class FoundState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                                   input_keys=['headings'])

    def execute(self, userdata):
        #TODO: Lock on to confirmed heading and go towards it
        rospy.loginfo('locking on and going')
        for heading in userdata.headings:
            print heading
        # Just take the first heading for now
        actionClient = actionlib.SimpleActionClient('search', ControllerAction)
        actionClient.wait_for_server()

        goal = PID_Controller.msg.ControllerGoal()
        goal.heading_setpoint = userdata.headings[0]
        goal.depth_setpoint = self.depth

        actionClient.sendGoal(goal)
        actionClient.waitForResult()

        return 'succeeded'



'''
Main
'''
if __name__ == '__main__':
    rospy.init_node('lane_marker_detector', anonymous=True)
    loopRateHz = rospy.get_param('~loopHz', 20)
    imageTopic = rospy.get_param('~image', '/bottomcam/camera/image_color')
    depthTopic = rospy.get_param('~depth', '/depth')
    compassTopic = rospy.get_param('~compass', '/euler')
    expectedLanes = rospy.get_param('~lanes', 1)

    # Set up param configuration window
    def configCallback(config, level):
        for param in params:
            params[param] = config[param]
        return config
    srv = Server(LaneMarkerDetectorConfig, configCallback)

    camdebug = CamDebug('lane_marker_detector', debugOn=DEBUG)


    global rosRate
    rosRate = rospy.Rate(loopRateHz)
    def gotRosFrame(rosImage): laneDetector.gotRosFrame(rosImage)
    def gotHeading(msg): laneDetector.gotHeading(msg)
    def gotDepth(msg): laneDetector.gotDepth(msg)
    rospy.Subscriber(imageTopic, Image, gotRosFrame)
    rospy.Subscriber(compassTopic, compass_data, gotHeading)
    rospy.Subscriber(depthTopic, depth, gotDepth)

    sm = smach.StateMachine(outcomes=['attempting', 'succeeded', 'aborted'])
    sm.userdata.expectedLanes = expectedLanes
    with sm:
        smach.StateMachine.add('SearchState', SearchState(),
                                transitions={'foundLane': 'ConfirmingState',
                                             'aborted': 'aborted'})
        smach.StateMachine.add('ConfirmingState', ConfirmingState(),
                                transitions={'confirmed': 'FoundState',
                                             'searchAgain': 'SearchState',
                                             'aborted': 'aborted'})
        smach.StateMachine.add('FoundState', FoundState(),
                                transitions={'succeeded': 'succeeded',
                                             'aborted': 'aborted'})

    outcome = sm.execute()

# vim: set sw=4 ts=4 expandtab:
