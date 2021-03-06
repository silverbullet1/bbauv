#!/usr/bin/env python2
'''
Code to identify RoboSub lane markers
'''

import roslib; roslib.load_manifest('Vision')
import rospy
from sensor_msgs.msg import Image
import bbauv_msgs
from bbauv_msgs.srv import *
from bbauv_msgs.msg import compass_data, depth

import actionlib
from bbauv_msgs.msg import ControllerAction, controller

import math
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from com.camdebug.camdebug import CamDebug
from com.lane_detector.lane_detector import LaneDetector
from com.utils.utils import norm_heading

from dynamic_reconfigure.server import Server
from Vision.cfg import LaneMarkerDetectorConfig

import smach
import smach_ros


TEST_MODE = False
DEBUG = True
#HACK:
firstRun = True
firstRunAction = True
camdebug = None
laneDetector = None
depth_setpoint = 0.5

test_heading = 0

isAborted = False

params = {  'hueLow':0, 'hueHigh':0,
            'satLow':0, 'satHigh':0,
            'valLow':0, 'valHigh':0,
            'contourMinArea':0,
            'houghThreshold':0,
            'houghMinLineLength':0,
            'houghMaxLineGap':0,
            'epsilonX':0, 'epsilonY':0,
            'forwardK':0, 'sidemoveK':0,
            'goalWait':0,
            'discardTimeout':0,
            'minVariance':0
} # Dynamic reconfigure params; see LaneMarkerDetector.cfg


class MedianFilter:
    def __init__(self, sampleWindow=30):
        self.samples = []
        self.sampleWindow = sampleWindow
        self.lastSampled = rospy.Time()

    def newSample(self, sample):
        curTime = rospy.Time()
        # Discard previous samples if we only sampled them a long time ago
        if (curTime - self.lastSampled) > rospy.Duration.from_sec(params['discardTimeout']):
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
class Disengage(smach.State):
    def handle_srv(self, req):
        global depth_setpoint, isAborted

        print 'got a request!'
        if req.start_request:
            self.isStart = True
            self.inputHeading = req.start_ctrl.heading_setpoint
            self.expectedLanes = req.num_lanes
            self.outDir = 'left' if req.use_left else 'right'
            depth_setpoint = req.start_ctrl.depth_setpoint

            rospy.wait_for_service('mission_srv')
            global mission_srv
            mission_srv = rospy.ServiceProxy('mission_srv', vision_to_mission, headers={'id':'0'})
            rospy.loginfo('connected to mission_srv!')

            isAborted = False

        if req.abort_request:
            isAborted = True

        return mission_to_laneResponse(self.isStart, isAborted)

    def __init__(self):
        self.isStart = False
        smach.State.__init__(
                        self,
                        outcomes=['start_complete', 'killed'],
                        output_keys=['expectedLanes', 'inputHeading', 'outDir']
        )

    def execute(self, userdata):
        global laneDetector
        laneDetector = None

        self.isStart = TEST_MODE

        global firstRun
        if firstRun:
            firstRun = False

            srvServer = rospy.Service('lane_srv', mission_to_lane, self.handle_srv)
            rospy.loginfo('lane_srv initialized!')

        while not self.isStart:
            if rospy.is_shutdown(): return 'killed'
            rosRate.sleep()

        laneDetector = LaneDetector(params, camdebug)
        if not TEST_MODE:
            userdata.inputHeading = laneDetector.inputHeading = self.inputHeading
        else:
            userdata.inputHeading = test_heading
            self.expectedLanes = expectedLanes
            self.outDir = outDir

        userdata.expectedLanes = self.expectedLanes
        userdata.outDir = self.outDir # 'left' or 'right'

        rospy.loginfo('Looking for %d lanes, going to take %s lane' % (self.expectedLanes, self.outDir))
        return 'start_complete'


class Search(smach.State):
    def __init__(self):
        smach.State.__init__(
                        self,
                        outcomes=['search_complete', 'aborted', 'killed'],
                        input_keys=['expectedLanes', 'inputHeading', 'outDir'],
                        output_keys=['expectedLanes', 'inputHeading', 'outDir']
        )

    def execute(self, userdata):
        global mission_srv, actionClient
        rospy.loginfo('Searching for lanes')

        actionClient = actionlib.SimpleActionClient('LocomotionServer', ControllerAction)
        print 'waiting for action server...'
        actionClient.wait_for_server()
        print 'done'

        
        while len(laneDetector.foundLines) == 0:
            if rospy.is_shutdown(): return 'killed'
            if isAborted: return 'aborted'
            rosRate.sleep()

        actionClient.cancel_all_goals()
        rospy.sleep(1)

        if not TEST_MODE:
            mission_srv(search_request=True, task_complete_request=False, task_complete_ctrl=None)
        return 'search_complete'


class Stabilize(smach.State):
    def __init__(self):
        smach.State.__init__(
                        self,
                        outcomes=['foundLane', 'aborted', 'killed'],
                        input_keys=['expectedLanes', 'inputHeading', 'outDir'],
                        output_keys=['expectedLanes', 'inputHeading', 'outDir']
        )

    def execute(self, userdata):
        EPSILON_X, EPSILON_Y = params['epsilonX'], params['epsilonY']

        laneDetector.frameCallback = None

        rospy.loginfo('moving around to find lane')
        heading = laneDetector.heading
        while len(laneDetector.foundLines) < userdata.expectedLanes or abs(laneDetector.offset[0])>EPSILON_X or abs(laneDetector.offset[1])>EPSILON_Y:
            if rospy.is_shutdown(): return 'killed'
            if isAborted: return 'aborted'

            x_off = abs(laneDetector.offset[0])>EPSILON_X
            y_off = abs(laneDetector.offset[1])>EPSILON_Y

            if x_off or y_off:
                initService()

                goal = bbauv_msgs.msg.ControllerGoal(
                    heading_setpoint = heading,
                    depth_setpoint = depth_setpoint
                )
                if x_off:
                    goal.sidemove_setpoint = laneDetector.offset[0] * params['sidemoveK']
                if y_off:
                    goal.forward_setpoint = -laneDetector.offset[1] * params['forwardK']

                actionClient.send_goal(goal)
                print 'adjusting'
                actionClient.wait_for_result(
                    rospy.Duration.from_sec(params['goalWait'])
                )
                print 'done adjusting'

            rosRate.sleep()

        print 'found a line, steadying heading'
        goal = bbauv_msgs.msg.ControllerGoal(
                heading_setpoint = laneDetector.heading,
                depth_setpoint = depth_setpoint,
                forward_setpoint = 0,
                sidemove_setpoint = 0)
        actionClient.send_goal(goal)
        actionClient.wait_for_result(
            rospy.Duration.from_sec(params['goalWait'])
        )

        actionClient.cancel_all_goals()
        print 'done steadying'

        return 'foundLane'


class Confirm(smach.State):
    def __init__(self):
        smach.State.__init__(
                        self,
                        outcomes=['confirmed', 'search_again', 'aborted', 'killed'],
                        input_keys=['expectedLanes', 'inputHeading', 'outDir'],
                        output_keys=['expectedLanes', 'inputHeading', 'headings', 'outDir']
        )

    def execute(self, userdata):
        self.headings = []
        self.medianFilters = []
        for i in range(userdata.expectedLanes):
            self.medianFilters.append(MedianFilter(sampleWindow=10))

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
            if rospy.is_shutdown(): return 'killed'
            if isAborted: return 'aborted'
            if self.status == 'lost': return 'search_again'
            if all([medianFilter.getVariance() < params['minVariance'] for medianFilter in self.medianFilters]):
                userdata.headings = [m.median for m in self.medianFilters]
                return 'confirmed'

            rosRate.sleep()

        return 'search_again'


class Found(smach.State):
    def __init__(self):
        smach.State.__init__(
                        self,
                        outcomes=['succeeded','killed'],
                        input_keys=['expectedLanes', 'headings', 'outDir']
        )

    def execute(self, userdata):
        global mission_srv
        laneDetector.frameCallback = None

        # Lock on to confirmed heading and go towards it
        rospy.loginfo('locking on and going')
        rospy.loginfo('Headings: ' + str(userdata.headings))

        finalHeading = userdata.headings[0]
        if userdata.expectedLanes > 1:
            # Assume we only have up to two lanes for now
            angleDelta = userdata.headings[1] - userdata.headings[0]
            leftIndex = 0 if 0 < angleDelta < 180 or angleDelta < -180 else 1

            leftHeading, rightHeading = userdata.headings[leftIndex], userdata.headings[1-leftIndex]
            if userdata.outDir == 'left':
                finalHeading = leftHeading
            elif userdata.outDir == 'right':
                finalHeading = rightHeading

        rospy.loginfo("Using heading %f" % finalHeading)

        ctrl = controller()
        ctrl.depth_setpoint = depth_setpoint
        ctrl.heading_setpoint = finalHeading
        if not TEST_MODE:
            mission_srv(search_request=False, task_complete_request=True, task_complete_ctrl=ctrl)
        else:
            return 'killed'

        return 'succeeded'


def initService():
    global firstRunAction
    if firstRunAction:
        print "waiting for service"
        rospy.wait_for_service('set_controller_srv')
        set_controller_request = rospy.ServiceProxy('set_controller_srv',set_controller)
        rospy.wait_for_service('set_controller_srv')
        set_controller_request(True, True, True, True, True, False, False )
        print "set controller request"

        firstRunAction = False


'''
Main
'''
if __name__ == '__main__':
    global expectedLanes

    rospy.init_node('lane_marker_detector', anonymous=False)
    loopRateHz = rospy.get_param('~loopHz', 20)
    imageTopic = rospy.get_param('~image', '/bottomcam/camera/image_color')
    depthTopic = rospy.get_param('~depth', '/depth')
    compassTopic = rospy.get_param('~compass', '/euler')
    expectedLanes = rospy.get_param('~lanes', 1)
    outDir = rospy.get_param('~out', 'left')
    TEST_MODE = rospy.get_param('~testmode', False)
    test_heading = rospy.get_param('~heading', 0)

    # Set up param configuration window
    def configCallback(config, level):
        for param in params:
            params[param] = config[param]
        return config
    srv = Server(LaneMarkerDetectorConfig, configCallback)

    camdebug = CamDebug('Vision', debugOn=DEBUG)

    global inputHeading
    inputHeading = 0 #TODO: take in the input heading for the previous task

    global rosRate
    rosRate = rospy.Rate(loopRateHz)
    def gotRosFrame(rosImage):
        if laneDetector: laneDetector.gotRosFrame(rosImage)
    def gotHeading(msg):
        if laneDetector: laneDetector.gotHeading(msg)
    def gotDepth(msg):
        if laneDetector: laneDetector.gotDepth(msg)
    rospy.Subscriber(imageTopic, Image, gotRosFrame)
    rospy.Subscriber(compassTopic, compass_data, gotHeading)
    rospy.Subscriber(depthTopic, depth, gotDepth)

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'killed'])
    with sm:
        smach.StateMachine.add(
                        'DISENGAGED',
                        Disengage(),
                        transitions={'start_complete':'SEARCH',
                                     'killed': 'killed'}
        )
        smach.StateMachine.add(
                        'SEARCH',
                        Search(),
                        transitions={'search_complete':'STABILIZE',
                                     'aborted': 'DISENGAGED',
                                     'killed': 'killed'}
        )
        smach.StateMachine.add(
                        'STABILIZE',
                        Stabilize(),
                        transitions={'foundLane': 'CONFIRM',
                                     'aborted': 'DISENGAGED',
                                     'killed': 'killed'}
        )
        smach.StateMachine.add(
                        'CONFIRM',
                        Confirm(),
                        transitions={'confirmed': 'FOUND',
                                     'search_again': 'STABILIZE',
                                     'aborted': 'DISENGAGED',
                                     'killed': 'killed'}
        )
        smach.StateMachine.add(
                        'FOUND',
                        Found(),
                        transitions={'succeeded': 'DISENGAGED',
                                     'killed': 'killed'}
        )

    sis = smach_ros.IntrospectionServer('mission_server', sm, '/MISSION/LANE_GATE')
    sis.start()

    try:
        outcome = sm.execute()
    except Exception, e:
        print e
        rospy.signal_shutdown('Shutting down due to exception.')

    rospy.signal_shutdown('Shutting down request')

# vim: set sw=4 ts=4 expandtab:
