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

#TODO: Use competition colours
TARGET_COLORS = ['redLed'] #NOTE: use 'redLed' for the red LED

# GLOBALS
TEST_MODE = False
DEBUG = True
firstRun = True
firstRunAction = True
isAborted = False
camdebug = None
actionClient = None
lightDetector = None
input_heading = 0
depth_setpoint = 0.5
cur_heading = 0

SIDEMOVE_SETPOINT = 3

#HACK
MAX_BUMPS = 2
bumpCount = 0

# For adjustments
X_MIN = 270
X_MAX = 370
W_MIN = 120
W_MAX = 300

#HACK: use a lock to prevent race conditions
lock = threading.Lock()

# dynamic_reconfigure params; see TrafficLight.cfg
params = { 'minBuoyRadius': 0, 'contourMinArea': 0,
           'redHueLow': 0, 'redHueHigh': 0, 'redSatLow': 0, 'redSatHigh': 0, 'redValLow': 0, 'redValHigh': 0,
           'redLedHueLow': 0, 'redLedHueHigh': 0, 'redLedSatLow': 0, 'redLedSatHigh': 0, 'redLedValLow': 0, 'redLedValHigh': 0, 
           'yellowHueLow': 0, 'yellowHueHigh': 0, 'yellowSatLow': 0, 'yellowSatHigh': 0, 'yellowValLow': 0, 'yellowValHigh': 0,
           'greenHueLow': 0, 'greenHueHigh': 0, 'greenSatLow': 0, 'greenSatHigh': 0, 'greenValLow': 0, 'greenValHigh': 0,
           'ledXMin': 0, 'ledXMax': 0, 'ledWMin': 0, 'ledWMax': 0,
           'bumpTime': 0, 'bumpK': 0,
           'farForwardK': 0, 'farSideK': 0,
           'nearForwardK': 0, 'nearSideK': 0
}


def initService():
    global firstRunAction, locomotion_mode_request
    if firstRunAction:
        print "waiting for service"
        rospy.wait_for_service('set_controller_srv')
        set_controller_request = rospy.ServiceProxy('set_controller_srv',set_controller)
        rospy.wait_for_service('set_controller_srv')
        set_controller_request(True, True, True, True, True, False, False)
        print "set controller request"
        locomotion_mode_request = rospy.ServiceProxy('locomotion_mode_srv',locomotion_mode)
        locomotion_mode_request(False,False)

        firstRunAction = False

        global actionClient
        actionClient = actionlib.SimpleActionClient('LocomotionServer', ControllerAction)
        print 'waiting for action server...'
        actionClient.wait_for_server()
        print 'done'


def getLedAdjustment():
    if lightDetector and lightDetector.colorsFound:
        centroid = lightDetector.colorsFound[0][1]
        rect = lightDetector.colorsFound[0][2]

        if X_MAX < centroid[0]:
            return 'right'
        elif centroid[0] < X_MIN:
            return 'left'

        if rect[3] < W_MIN:
            return 'forward'
        elif rect[3] > W_MAX:
            return 'back'


'''
Performs correction to keep target in centre
'''
class Correction:
    def __init__(self, FORWARD_K=-5, SIDE_K=8.0, DEPTH_K=0.4, ANGLE_K=-0.4,
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

        self.hoverHeading = input_heading if not TEST_MODE else cur_heading

    def correct(self):
        hoverDepth = depth_setpoint
        offsets = {}

        def isOutOfPlace():
            offsets['x'] = offsets['y'] = 0
            offsets['size'] = offsets['angle'] = 0
            if not lightDetector.buoyDetected:
                return True

            x,y = lightDetector.redCentre
            #HACK: shift y by a bit to compensate
            y -= 50
            rad = lightDetector.redRadius
            w = h = 2*rad
            H,W = lightDetector.shape[0:2]

            sizeRatio = 2*math.pi*rad*rad/float(W*H)
#            sizeMidPt = (self.MIN_SIZE + self.MAX_SIZE) * 0.5
#            if sizeRatio < self.MIN_SIZE or sizeRatio > self.MAX_SIZE:
#                offsets['size'] = sizeRatio - sizeMidPt
            if sizeRatio < self.MIN_SIZE:
                offsets['size'] = sizeRatio - self.MIN_SIZE

            print 'vals:', { 'x': x, 'y': y, 'w': w, 'h': h, 'sizeRatio': sizeRatio }

            offsets['x'] = clamp(x/float(W) - 0.5, -1, 1)
            offsets['y'] = clamp(y/float(H) - 0.5, -1, 1)

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
                r = lightDetector.redRadius

                factor = clamp(1 - r/float(H), 0, 1) # attenuation factor
                hoverDepth = depth_setpoint + factor * self.DEPTH_K * offsets['y']
                goal = bbauv_msgs.msg.ControllerGoal(
                        heading_setpoint = self.hoverHeading,
                        depth_setpoint = hoverDepth
                )
                waitTime = rospy.Duration(1,0)
            elif abs(offsets['x']) > self.EPSILON_X:
                print("Correcting horizontal")
                x,y = lightDetector.redCentre
                r = lightDetector.redRadius

                factor = clamp(1 - r/float(W), 0, 1)
                goal = bbauv_msgs.msg.ControllerGoal(
                        heading_setpoint = self.hoverHeading,
                        depth_setpoint = hoverDepth,
                        sidemove_setpoint = factor * self.SIDE_K * offsets['x']
                )
                waitTime = rospy.Duration(2,0)
            elif offsets['size']:
                print ("Correcting distance")
                goal = bbauv_msgs.msg.ControllerGoal(
                        heading_setpoint = self.hoverHeading,
                        depth_setpoint = hoverDepth,
                        forward_setpoint = self.FORWARD_K * offsets['size']
                )
                waitTime = rospy.Duration(2,0)

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
        global input_heading, depth_setpoint, isAborted

        print 'got a request!'
        if req.start_request:
            input_heading  = req.start_ctrl.heading_setpoint
            depth_setpoint = req.start_ctrl.depth_setpoint

            rospy.wait_for_service('mission_srv')
            global mission_srv
            mission_srv = rospy.ServiceProxy('mission_srv', vision_to_mission, headers={'id':1})
            rospy.loginfo('connected to mission_srv!')

            isAborted = False
            self.isStart = True

        if req.abort_request:
            isAborted = True

        return mission_to_visionResponse(self.isStart, isAborted)

    def __init__(self):
        self.isStart = False
        smach.State.__init__(
                        self,
                        outcomes=['start_complete', 'killed'],
                        output_keys=['targetColors']
        )

    def execute(self, userdata):
        global lightDetector
        lightDetector = None

        global firstRun
        if firstRun:
            firstRun = False

            rospy.Service('traffic_srv', mission_to_vision, self.handle_srv)
            rospy.loginfo('traffic_srv initialized!')

        self.isStart = TEST_MODE

        while not self.isStart:
            if rospy.is_shutdown(): return 'killed'
            rosRate.sleep()

        lightDetector = TrafficLight(params, lock, camdebug)
        userdata.targetColors = TARGET_COLORS

        return 'start_complete'


'''
Detect the traffic lights (either red, yellow or green)
'''
class Search(smach.State):
    def __init__(self):
        smach.State.__init__(
                        self,
                        outcomes=['search_complete', 'aborted', 'killed'],
                        input_keys=['targetColors'],
                        output_keys=['targetColors']
        )

    def execute(self, userdata):
        if False: #DEBUG
            while not rospy.is_shutdown():
                rospy.sleep(1)
            return 'killed'
        else:
            #TODO: detect all the lights
            while not lightDetector.buoyDetected:
                if rospy.is_shutdown(): return 'killed'
                if isAborted: return 'aborted'
                rosRate.sleep()

            if not TEST_MODE:
                mission_srv(search_request=True, task_complete_request=False, task_complete_ctrl=None)

            return 'search_complete'


'''
Keep all three in nice position
'''
class Stabilize(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'killed'],
                             input_keys=['targetColors'],
                             output_keys=['targetColors','heading'])

    def execute(self, userdata):
        initService()

        correction = Correction(FORWARD_K=-params['farForwardK'], SIDE_K=params['farSideK'],
                                MIN_SIZE=0.111, MAX_SIZE=0.3, timeout=rospy.Duration(60,0))
        result = correction.correct()
        if result in ['aborted', 'killed']:
            return result

        userdata.heading = correction.hoverHeading
        return 'succeeded'


'''
Move to buoy
'''
class MoveToBuoy(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'killed'],
                             input_keys=['heading','targetColors'],
                             output_keys=['heading', 'targetColors'])

    def execute(self, userdata):
        correction = Correction(FORWARD_K=-params['nearForwardK'], SIDE_K=params['nearSideK'],
                                MIN_SIZE=0.19, MAX_SIZE=0.3, timeout=rospy.Duration(60,0))
        correction.hoverHeading = userdata.heading
        result = correction.correct()
        if result in ['aborted', 'killed']:
            return result

        return 'succeeded'


'''
Do a bump
'''
class BumpIt(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['bumped', 'done', 'aborted', 'killed'],
                             input_keys=['heading', 'targetColors'],
                             output_keys=['targetColors', 'sidemoveDir'])

    def execute(self, userdata):
        hoverDepth = depth_setpoint
        hoverHeading = userdata.heading

        BUMP_FORWARD = params['bumpK']

        for setpoint in [BUMP_FORWARD, -BUMP_FORWARD]:
            if rospy.is_shutdown(): return 'killed'
            if isAborted: return 'aborted'

            goal = bbauv_msgs.msg.ControllerGoal(
                    heading_setpoint = hoverHeading,
                    depth_setpoint = hoverDepth,
                    forward_setpoint = setpoint
            )
            actionClient.send_goal(goal)
            actionClient.wait_for_result(rospy.Duration.from_sec(params['bumpTime']))

        actionClient.cancel_all_goals()

        userdata.sidemoveDir = 'right'
        #HACK: shift upwards to better hit LEDs

        if userdata.targetColors:
#            global depth_setpoint
#            depth_setpoint -= 0.2
            return 'bumped'
        else:
            # Just want to hit the red buoy
            return 'done'


'''
Sidemove to next light and bump it
'''
#NOTE: assumption: red buoy is left of the LEDs
class SidemoveToBuoy(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['reached', 'done', 'aborted', 'killed'],
                             input_keys=['heading','sidemoveDir', 'targetColors'],
                             output_keys=['heading', 'targetColors'])

    def execute(self, userdata):
        # Move to the right until no colors found,
        # then continue moving until we see a color

        #TODO: set locomotion_mode
        goal = bbauv_msgs.msg.ControllerGoal(
                heading_setpoint = userdata.heading,
                depth_setpoint = depth_setpoint,
                sidemove_setpoint = SIDEMOVE_SETPOINT if userdata.sidemoveDir == 'right' else -SIDEMOVE_SETPOINT
        )
        actionClient.send_goal(goal)

        noBuoyCount = 0 # number of consecutive frames without seeing a buoy
        while noBuoyCount < 5:
            if rospy.is_shutdown(): return 'killed'
            if isAborted: return 'aborted'

            if lightDetector.colorsFound:
                noBuoyCount = 0
            else:
                noBuoyCount += 1
            rospy.sleep(0.05)

        buoyCount = 0 # number of consecutive frames seeing a buoy
        while buoyCount < 5:
            if rospy.is_shutdown(): return 'killed'
            if isAborted: return 'aborted'

            if not lightDetector.colorsFound:
                buoyCount = 0
            else:
                buoyCount += 1
            rospy.sleep(0.05)

        #TODO: centre the buoy
        while True:
            if rospy.is_shutdown(): return 'killed'
            if isAborted: return 'aborted'

            if lightDetector.colorsFound:
                centroid = lightDetector.colorsFound[0][1]
                if (userdata.sidemoveDir == 'right' and centroid[0] < X_MAX) or (userdata.sidemoveDir == 'left' and centroid[0] > X_MIN):
                    actionClient.cancel_all_goals()
                    break
            rospy.sleep(0.05)

        if userdata.targetColors:
            return 'reached'
        else:
            return 'done'

'''
Bump until a certain color
'''
class BumpToColor(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['bumped', 'adjustment', 'aborted', 'killed'],
                             input_keys=['heading', 'targetColors'],
                             output_keys=['adjustment', 'targetColors', 'sidemoveDir'])

    def execute(self, userdata):
        global bumpCount

        hoverDepth = depth_setpoint
        hoverHeading = userdata.heading

        BUMP_FORWARD = params['bumpK']

        while True:
            if rospy.is_shutdown(): return 'killed'
            if isAborted: return 'aborted'

            if lightDetector.colorsFound:
                centroid = lightDetector.colorsFound[0][1]
                # If out of place, adjust
                adjustment = getLedAdjustment()
                if adjustment:
                    userdata.adjustment = adjustment
                    return 'adjustment'

            bumpEnd = False
            if bumpCount < MAX_BUMPS:
                for setpoint in [BUMP_FORWARD, -BUMP_FORWARD]:
                    if rospy.is_shutdown(): return 'killed'
                    if isAborted: return 'aborted'

                    goal = bbauv_msgs.msg.ControllerGoal(
                            heading_setpoint = hoverHeading,
                            depth_setpoint = hoverDepth,
                            forward_setpoint = setpoint
                    )
                    actionClient.send_goal(goal)
                    actionClient.wait_for_result(rospy.Duration.from_sec(params['bumpTime']))
                actionClient.cancel_all_goals()
                bumpCount += 1
                rospy.sleep(1) # wait a bit before checking LED color
            else:
                bumpEnd = True

            if lightDetector.colorsFound and lightDetector.colorsFound[0][0] == userdata.targetColors[0]:
                bumpEnd = True

            if bumpEnd:
                bumpCount = 0
                remainingColors = userdata.targetColors[1:]
                userdata.targetColors = remainingColors
                break

        print 'remainingColors', remainingColors
        userdata.sidemoveDir = 'right' if remainingColors else 'left' # Go back to middle buoy

        return 'bumped'

'''
Centre LED on the camera feed
'''
class CenterOnLED(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'killed'],
                             input_keys=['adjustment', 'heading', 'targetColors'],
                             output_keys=['heading', 'targetColors'])

    def execute(self, userdata):
        sidemove_setpoint = 0
        if userdata.adjustment == 'right':
            sidemove_setpoint = SIDEMOVE_SETPOINT
        elif userdata.adjustment == 'left':
            sidemove_setpoint = -SIDEMOVE_SETPOINT

        forward_setpoint = 0

        goal = bbauv_msgs.msg.ControllerGoal(
                heading_setpoint = userdata.heading,
                depth_setpoint = depth_setpoint,
                sidemove_setpoint = sidemove_setpoint,
                forward_setpoint = forward_setpoint
        )
        actionClient.send_goal(goal)

        while True:
            if rospy.is_shutdown(): return 'killed'
            if isAborted: return 'aborted'

            if lightDetector.colorsFound:
                centroid = lightDetector.colorsFound[0][1]
                rect = lightDetector.colorsFound[0][2]

                if (userdata.adjustment == 'right' and centroid[0] < X_MAX) or (userdata.adjustment == 'left' and centroid[0] > X_MIN):
                    actionClient.cancel_all_goals()
                    break

                if (userdata.adjustment == 'forward' and rect[3] > W_MIN) or (userdata.adjustment == 'back' and rect[3] < W_MAX):
                    actionClient.cancel_all_goals()
                    break
            rospy.sleep(0.05)

        return 'succeeded'

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

        ctrl = controller()
        ctrl.depth_setpoint = depth_setpoint
        ctrl.heading_setpoint = input_heading if not TEST_MODE else cur_heading
        if not TEST_MODE:
            mission_srv(search_request=False, task_complete_request=True, task_complete_ctrl=ctrl)

        return 'succeeded'


'''
Main
'''
if __name__ == '__main__':
    rospy.init_node('traffic_light_task', anonymous=False)
    loopRateHz = rospy.get_param('~loopHz', 20)
    imageTopic = rospy.get_param('~image', '/stereo_camera/left/image_rect_color')
    depthTopic = rospy.get_param('~depth', '/depth')
    compassTopic = rospy.get_param('~compass', '/euler')
    TEST_MODE = rospy.get_param('~testmode', False)

    # Set up param configuration window
    def configCallback(config, level):
        global X_MIN, X_MAX, W_MIN, W_MAX
        for param in params:
            params[param] = config[param]
        X_MIN = params['ledXMin']
        X_MAX = params['ledXMax']
        W_MIN = params['ledWMin']
        W_MIN = params['ledWMax']
        return config
    srv = Server(TrafficLightConfig, configCallback)

    camdebug = CamDebug('Vision', debugOn=DEBUG)

    global rosRate
    rosRate = rospy.Rate(loopRateHz)
    def gotRosFrame(rosImage):
        if lightDetector: lightDetector.gotRosFrame(rosImage)
    def gotHeading(msg):
        global cur_heading
        cur_heading = msg.yaw
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
                        transitions={'succeeded': 'MOVE_TO_BUOY',
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
                        transitions={'done': 'DONE',
                                     'bumped': 'SIDEMOVE_TO_BUOY',
                                     'aborted': 'DISENGAGED',
                                     'killed': 'killed'}
        )
        smach.StateMachine.add(
                        'SIDEMOVE_TO_BUOY',
                        SidemoveToBuoy(),
                        transitions={'reached': 'BUMP_TO_COLOR',
                                     'done': 'DONE',
                                     'aborted': 'DISENGAGED',
                                     'killed': 'killed'}
        )
        smach.StateMachine.add(
                        'BUMP_TO_COLOR',
                        BumpToColor(),
                        transitions={'bumped': 'SIDEMOVE_TO_BUOY',
                                     'adjustment': 'CENTER_ON_LED',
                                     'aborted': 'DISENGAGED',
                                     'killed': 'killed'}
        )
        smach.StateMachine.add(
                        'CENTER_ON_LED',
                        CenterOnLED(),
                        transitions={'succeeded': 'BUMP_TO_COLOR',
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
        rospy.signal_shutdown('Shutting down due to exception.')

    rospy.signal_shutdown('Task completed')

# vim: set sw=4 ts=4 expandtab:
