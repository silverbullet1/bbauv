#!/usr/bin/env python2
'''
Identify traffic lights
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
import threading

from com.camdebug.camdebug import CamDebug
from com.utils.utils import *
from com.trafficlight.traffic_light_detector import TrafficLight

from dynamic_reconfigure.server import Server
from Vision.cfg import TrafficLightConfig

import smach
import smach_ros

# GLOBALS
DEBUG = True
firstRun = True
firstRunAction = True
isAborted = False
camdebug = None
lightDetector = None
depth_setpoint = 0.5
cur_heading = 0

#HACK: use a lock to prevent race conditions
lock = threading.Lock()

# dynamic_reconfigure params; see TrafficLight.cfg
params = { 'contourMinArea': 0,
           'redHueLow': 0, 'redHueHigh': 0, 'redSatLow': 0, 'redSatHigh': 0, 'redValLow': 0, 'redValHigh': 0,
           'yellowHueLow': 0, 'yellowHueHigh': 0, 'yellowSatLow': 0, 'yellowSatHigh': 0, 'yellowValLow': 0, 'yellowValHigh': 0,
           'greenHueLow': 0, 'greenHueHigh': 0, 'greenSatLow': 0, 'greenSatHigh': 0, 'greenValLow': 0, 'greenValHigh': 0
}


def initService():
    global firstRunAction
    if firstRunAction:
        print "waiting for service"
        rospy.wait_for_service('set_controller_srv')
        set_controller_request = rospy.ServiceProxy('set_controller_srv',set_controller)
        rospy.wait_for_service('set_controller_srv')
        set_controller_request(True, True, True, True, False, False, False)
        print "set controller request"

        firstRunAction = False


'''
Performs correction to keep target in centre
'''
class Correction:
    def __init__(self, FORWARD_K=-12, SIDE_K=10.0, DEPTH_K=1.0, ANGLE_K=-0.4,
                 EPSILON_X=0.08, EPSILON_Y=0.08, EPSILON_ANGLE=4,
                 MIN_SIZE=0.033, MAX_SIZE=0.6, timeout=None):

        self.FORWARD_K = FORWARD_K
        self.SIDE_K = SIDE_K
        self.DEPTH_K = DEPTH_K
        self.ANGLE_K = ANGLE_K

        self.EPSILON_X = EPSILON_X
        self.EPSILON_Y = EPSILON_Y
        self.EPSILON_ANGLE = EPSILON_ANGLE
        self.MIN_SIZE = MIN_SIZE
        self.MAX_SIZE = MAX_SIZE

        self.timeout = timeout

    def correct(self):
        hoverDepth = depth_setpoint
        hoverHeading = cur_heading
        offsets = {}

        def isOutOfPlace():
            offsets['x'] = offsets['y'] = 0
            offsets['size'] = offsets['angle'] = 0
            if not lightDetector.buoyDetected:
                return True

            x,y = lightDetector.redCentre
            rad = lightDetector.redRadius
            w = h = 2*rad
            H,W = lightDetector.shape[0:2]

            if self.target == 'hole':
                x -= self.hole_offset[0]
                y -= self.hole_offset[1]

            sizeRatio = w*h/float(W*H)
            sizeMidPt = (self.MIN_SIZE + self.MAX_SIZE) * 0.5
            if sizeRatio < self.MIN_SIZE or sizeRatio > self.MAX_SIZE:
                offsets['size'] = sizeRatio - sizeMidPt

            print 'vals:', { 'x': x, 'y': y, 'w': w, 'h': h, 'sizeRatio': sizeRatio }

            offsets['x'] = clamp((x + w/2)/float(W) - 0.5, -1, 1)
            offsets['y'] = clamp((y + h/2)/float(H) - 0.5, -1, 1)

#            pts = lightDetector.bigQuad
#            angles = [
#                math.degrees(math.atan2(pts[1][1]-pts[0][1], pts[1][0]-pts[0][0])),
#                math.degrees(math.atan2(pts[2][1]-pts[3][1], pts[2][0]-pts[3][0]))
#            ]
#            offsets['angle'] = clamp(angles[0] - angles[1], -20, 20)

            if abs(offsets['x']) > self.EPSILON_X or abs(offsets['y']) > self.EPSILON_Y:
                return True
            if offsets['size'] or abs(offsets['angle']) > self.EPSILON_ANGLE:
                return True

            return False

        startTime = rospy.Time.now()

        while True:
            if rospy.is_shutdown(): return 'killed'
            if isAborted: return 'aborted'

            if self.timeout is not None and rospy.Time.now() - startTime > self.timeout:
                rospy.loginfo('correction timeout!')
                break

            lock.acquire() #HACK

            if not isOutOfPlace():
                lock.release() #HACK
                break

            print 'offsets:', offsets

            H,W = lightDetector.shape[0:2]

            goal = None
            waitTime = None

            if abs(offsets['y']) > self.EPSILON_Y:
                print("Correcting vertical")
                x,y = lightDetector.redCentre
                w = h = 2*lightDetector.redRadius

                factor = 1 - h/float(H) # attenuation factor
                hoverDepth = depth_setpoint + factor * self.DEPTH_K * offsets['y']
                goal = bbauv_msgs.msg.ControllerGoal(
                        heading_setpoint = hoverHeading,
                        depth_setpoint = hoverDepth
                )
                waitTime = rospy.Duration(2,0)
            elif abs(offsets['x']) > self.EPSILON_X:
                print("Correcting horizontal")
                x,y = lightDetector.redCentre
                w = h = 2*lightDetector.redRadius

                factor = 1 - w/float(W)
                goal = bbauv_msgs.msg.ControllerGoal(
                        heading_setpoint = hoverHeading,
                        depth_setpoint = hoverDepth,
                        sidemove_setpoint = factor * self.SIDE_K * offsets['x']
                )
                waitTime = rospy.Duration(3,0)
            elif abs(offsets['angle']) > self.EPSILON_ANGLE:
                print ("Correcting heading")
                hoverHeading = norm_heading(lightDetector.heading + self.ANGLE_K * offsets['angle'])

                goal = bbauv_msgs.msg.ControllerGoal(
                        heading_setpoint = hoverHeading,
                        depth_setpoint = hoverDepth,
                        sidemove_setpoint = math.copysign(3.0, offsets['angle'])
                )
                waitTime = rospy.Duration(3,0)
            elif offsets['size']:
                print ("Correcting distance")
                goal = bbauv_msgs.msg.ControllerGoal(
                        heading_setpoint = hoverHeading,
                        depth_setpoint = hoverDepth,
                        forward_setpoint = self.FORWARD_K * offsets['size']
                )
                waitTime = rospy.Duration(3,0)

            lock.release() #HACK

            if goal is not None:
                actionClient.send_goal(goal)
                actionClient.wait_for_result(waitTime)
                print("got result")

            rosRate.sleep()



'''
States
'''
class Disengage(smach.State):
    def handle_srv(self, req):
        global depth_setpoint, isAborted

        print 'got a request!'
        if req.start_request:
            self.inputHeading = req.start_ctrl.heading_setpoint
            depth_setpoint = req.start_ctrl.depth_setpoint

            rospy.wait_for_service('mission_srv')
            global mission_srv
            mission_srv = rospy.ServiceProxy('mission_srv', vision_to_mission)
            rospy.loginfo('connected to mission_srv!')

            isAborted = False
            self.isStart = True

        if req.abort_request:
            isAborted = True

        return mission_to_visionResponse(self.isStart, isAborted)

    def __init__(self):
        self.isStart = True #TODO: return false in actual mission
        smach.State.__init__(
                        self,
                        outcomes=['start_complete', 'killed'],
                        output_keys=[]
        )

    def execute(self, userdata):
        global lightDetector
        lightDetector = None

        self.isStart = False

        while not self.isStart:
            if rospy.is_shutdown(): return 'killed'
            rosRate.sleep()

        lightDetector = TrafficLight(params, lock, camdebug)
        lightDetector.inputHeading = self.inputHeading

        return 'start_complete'


'''
Detect the traffic lights (either red, yellow or green)
'''
class Search(smach.State):
    def __init__(self):
        smach.State.__init__(
                        self,
                        outcomes=['search_complete', 'aborted', 'killed'],
                        input_keys=[],
                        output_keys=[]
        )

    def execute(self, userdata):
        global mission_srv

        #TODO: detect all the lights
        while not lightDetector.buoyDetected:
            if rospy.is_shutdown(): return 'killed'
            if isAborted: return 'aborted'
            rosRate.sleep()

#        mission_srv(search_request=True, task_complete_request=False, task_complete_ctrl=None)
        return 'search_complete'


'''
Keep all three in nice position
'''
class Stabilize(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['found', 'aborted', 'killed'],
                             input_keys=['targetColors'],
                             output_keys=['targetColors'])

    def execute(self, userdata):
        initAction()

        correction = Correction(timeout=rospy.Duration(90,0))
        result = correction.correct()
        if result in ['aborted', 'killed']:
            return result

        return 'found'


'''
Move to buoy
'''
class MoveToBuoy(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'killed'],
                             input_keys=['targetColors'],
                             output_keys=['targetColors'])

    def execute(self, userdata):
        correction = Correction(timeout=rospy.Duration(90,0))
        result = correction.correct()
        if result in ['aborted', 'killed']:
            return result

        return 'found'


'''
Do a bump
'''
class BumpIt(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['bumped', 'aborted', 'killed'],
                             input_keys=['targetColors'],
                             output_keys=['targetColors'])

    def execute(self, userdata):
        hoverDepth = depth_setpoint
        hoverHeading = cur_heading

        BUMP_FORWARD = 10

        for setpoint in [BUMP_FORWARD, -BUMP_FORWARD]:
            if rospy.is_shutdown(): return 'killed'
            if isAborted: return 'aborted'

            goal = bbauv_msgs.msg.ControllerGoal(
                    heading_setpoint = hoverHeading,
                    depth_setpoint = hoverDepth,
                    forward_setpoint = setpoint
            )
            actionClient.send_goal(goal)
            actionClient.wait_for_result(rospy.Duration(2,0))

        return 'bumped'


'''
Move to next light and bump it
'''
#TODO: wait until light turns to correct colour, then bump it


'''
Done
'''
class Done(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted'],
                             input_keys=[],
                             output_keys=[])

    def execute(self, userdata):
        if isAborted: return 'aborted'
        return 'succeeded'


'''
Main
'''
if __name__ == '__main__':
    rospy.init_node('traffic_light_task', anonymous=False)
    loopRateHz = rospy.get_param('~loopHz', 20)
    imageTopic = rospy.get_param('~image', '/stereo_camera/right/image_rect_color')
    depthTopic = rospy.get_param('~depth', '/depth')
    compassTopic = rospy.get_param('~compass', '/euler')

    # Set up param configuration window
    def configCallback(config, level):
        for param in params:
            params[param] = config[param]
        return config
    srv = Server(TrafficLightConfig, configCallback)

    camdebug = CamDebug('Vision', debugOn=DEBUG)

    global inputHeading
    inputHeading = 0 #TODO: take in the input heading for the previous task

    global rosRate
    rosRate = rospy.Rate(loopRateHz)
    def gotRosFrame(rosImage):
        if lightDetector: lightDetector.gotRosFrame(rosImage)
    def gotHeading(msg):
        global cur_heading
        cur_heading = msg.yaw
        if lightDetector: lightDetector.gotHeading(msg)
    def gotDepth(msg):
        global depth_setpoint
        depth_setpoint = msg.depth
    rospy.Subscriber(imageTopic, Image, gotRosFrame)
    rospy.Subscriber(compassTopic, compass_data, gotHeading)
    rospy.Subscriber(depthTopic, depth, gotDepth)

    sm = smach.StateMachine(outcomes=['succeeded', 'killed'])
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
                        transitions={'found': 'MOVE_TO_BUOY',
                                     'aborted': 'DISENGAGED',
                                     'killed': 'killed'}
        )
        smach.StateMachine.add(
                        'MOVE_TO_BUOY',
                        MoveToBuoy(),
                        transitions={'succeeded': 'BUMP',
                                     'aborted': 'DISENGAGED',
                                     'killed': 'killed'}
        )
        smach.StateMachine.add(
                        'BUMP',
                        BumpIt(),
                        transitions={'bumped': 'DONE',
                                     'aborted': 'DISENGAGED',
                                     'killed': 'killed'}
        )
        smach.StateMachine.add(
                        'DONE',
                        Done(),
                        transitions={'succeeded': 'succeeded',
                                     'aborted': 'DISENGAGED'}
        )

    sis = smach_ros.IntrospectionServer('trafficlight_server', sm, '/MISSION/TRAFFIC_LIGHT')
    sis.start()

    try:
        outcome = sm.execute()
    except Exception, e:
        print e
        rospy.shutdown_reason('Shutting down due to exception.')

# vim: set sw=4 ts=4 expandtab:
