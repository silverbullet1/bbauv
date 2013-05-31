#!/usr/bin/env python2
'''
Code to identify RoboSub lane markers
'''

import roslib; roslib.load_manifest('Vision')
import rospy
from sensor_msgs.msg import Image
from bbauv_msgs.msg import compass_data, depth

import actionlib
import PID_Controller.msg
from PID_Controller.msg import ControllerAction

import math
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from com.camdebug.camdebug import CamDebug
from com.lane_detector.lane_detector import LaneDetector

from dynamic_reconfigure.server import Server
from Vision.cfg import LaneMarkerDetectorConfig

import smach


DEBUG = True
#HACK:
camdebug = None
laneDetector = None

params = { 'hueLow':0, 'hueHigh':0, 'satLow':0, 'satHigh':0, 'valLow':0, 'valHigh':0, 'contourMinArea':0 } # Dynamic reconfigure params; see LaneMarkerDetector.cfg


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
                             input_keys=['expectedLanes', 'inputHeading'],
                             output_keys=['expectedLanes', 'inputHeading'])

    def execute(self, userdata):
        global laneDetector
        laneDetector = LaneDetector(params, camdebug)
        laneDetector.inputHeading = userdata.inputHeading

#        actionClient = actionlib.SimpleActionClient('LocomotionServer', ControllerAction)
#        print 'wait'
#        actionClient.wait_for_server()
#        print 'done'

        #TODO: Move around in a smarter way to keep all lanes on screen
        rospy.loginfo('moving around to find lane')
        while len(laneDetector.foundLines) < userdata.expectedLanes:
            if rospy.is_shutdown(): return 'aborted'

#            #TODO: use values related to spin rate
#            goal = PID_Controller.msg.ControllerGoal(
#                heading_setpoint = laneDetector.heading + 10,
#                depth_setpoint = laneDetector.depth,
#                forward_setpoint = 0,
#                sidemove_setpoint = 0)
#
#            actionClient.send_goal(goal)
#            actionClient.wait_for_result(rospy.Duration(3,0))
#            print 'done waiting for goal'

            rosRate.sleep()

        print 'found a line, steadying heading'
#        goal = PID_Controller.msg.ControllerGoal(
#                heading_setpoint = laneDetector.heading,
#                depth_setpoint = laneDetector.depth,
#                forward_setpoint = 0,
#                sidemove_setpoint = 0)
#        actionClient.send_goal(goal)
#        actionClient.wait_for_result(rospy.Duration(4,0))
        print 'done steadying'

        return 'foundLane'

class ConfirmingState(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['confirmed', 'searchAgain', 'aborted'],
                             input_keys=['expectedLanes', 'inputHeading'],
                             output_keys=['expectedLanes', 'inputHeading', 'headings'])

    def execute(self, userdata):
        self.headings = []
        self.medianFilters = []
        for i in range(userdata.expectedLanes):
            self.medianFilters.append(MedianFilter())

        global laneDetector
        laneDetector = LaneDetector(params, camdebug)
        laneDetector.inputHeading = userdata.inputHeading

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
            if all([medianFilter.getVariance() < 16 for medianFilter in self.medianFilters]):
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

        # Turn to heading
        goal = PID_Controller.msg.ControllerGoal(
                heading_setpoint = userdata.headings[0],
                depth_setpoint = laneDetector.depth,
                sidemove_setpoint = 0,
                forward_setpoint = 0)
        actionClient.send_goal(goal)
        actionClient.wait_for_result()

        # Move forward
        goal = PID_Controller.msg.ControllerGoal(
                heading_setpoint = userdata.headings[0],
                depth_setpoint = laneDetector.depth,
                sidemove_setpoint = 0,
                forward_setpoint = 5)
        actionClient.send_goal(goal)
        actionClient.wait_for_result()

        return 'succeeded'



'''
Main
'''
if __name__ == '__main__':
    rospy.init_node('lane_marker_detector', anonymous=False)
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

    inputHeading = 0 #TODO: take in the input heading for the previous task

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
    sm.userdata.inputHeading = inputHeading
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
