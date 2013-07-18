#!/usr/bin/env python2
'''
Code to identify RoboSub tollbooth and carry out task
'''

import roslib; roslib.load_manifest('Vision')
import rospy
from sensor_msgs.msg import Image
import bbauv_msgs
from bbauv_msgs.msg import compass_data, depth, manipulator
from bbauv_msgs.srv import *

import actionlib
from bbauv_msgs.msg import ControllerAction, controller

import math
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import threading

from com.camdebug.camdebug import CamDebug
from com.tollbooth.tollbooth import TollboothDetector
from com.utils.utils import *

from dynamic_reconfigure.server import Server
from Vision.cfg import TollboothConfig

import smach
import smach_ros


#TODO: use actual competition IDs
SINGLE_BOARD_MODE = False # Set to True if just going for a single color
COMPETITION_TARGETS = ['red','red'] if SINGLE_BOARD_MODE else ['red', 'blue']

# GLOBALS
TEST_MODE = True
DEBUG = True
camdebug = None
tollbooth = None
firstRun = True
firstRunAction = True
depth_setpoint = 0.5
cur_heading = 0
imageSub = None
gunSide = 'left'

isAborted = False
mission_srv = None

SWAP_EYE   = True    # Set this to False if you don't want to switch between eyes
currentEye = 'left'

#HACK: use a lock to prevent race conditions
lock = threading.Lock()

# dynamic_reconfigure params; see Tollbooth.cfg
params = { 'contourMinArea': 0,
           'red_hueLow': 0, 'red_hueHigh': 0, 'red_satLow': 0, 'red_satHigh': 0, 'red_valLow': 0, 'red_valHigh': 0,
           'blue_hueLow': 0, 'blue_hueHigh': 0, 'blue_satLow': 0, 'blue_satHigh': 0, 'blue_valLow': 0, 'blue_valHigh': 0,
           'yellow_hueLow': 0, 'yellow_hueHigh': 0, 'yellow_satLow': 0, 'yellow_satHigh': 0, 'yellow_valLow': 0, 'yellow_valHigh': 0,
           'green_hueLow': 0, 'green_hueHigh': 0, 'green_satLow': 0, 'green_satHigh': 0, 'green_valLow': 0, 'green_valHigh': 0,
           'depthGoalWait': 0, 'otherGoalWait': 0,
           'farForwardK':0, 'farSideK':0, 'farDepthK':0, 'farAngleK':0,
           'farEpsilonX':0, 'farEpsilonY':0, 'farEpsilonAngle':0,
           'farMinSize':0, 'farMaxSize':0, 'farTimeout':0,
           'nearForwardK':0, 'nearSideK':0, 'nearDepthK':0, 'nearAngleK':0,
           'nearEpsilonX':0, 'nearEpsilonY':0, 'nearEpsilonAngle':0,
           'nearMinSize':0, 'nearMaxSize':0, 'nearTimeout':0,
           'holeForwardK':0, 'holeSideK':0, 'holeDepthK':0, 'holeAngleK':0,
           'holeEpsilonX':0, 'holeEpsilonY':0, 'holeEpsilonAngle':0,
           'holeMinSize':0, 'holeMaxSize':0, 'holeTimeout':0,
           'chargeWait':0, 'fireWait':0
}


def initAction():
    global firstRunAction
    if firstRunAction:
        print "waiting for set_controller_srv"
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


# States
class Disengage(smach.State):
    def handle_srv(self, req):
        global depth_setpoint, isAborted

        print 'got a request!'
        if req.start_request:
            self.inputHeading = req.start_ctrl.heading_setpoint
            depth_setpoint = req.start_ctrl.depth_setpoint

            rospy.wait_for_service('mission_srv')
            global mission_srv
            mission_srv = rospy.ServiceProxy('mission_srv', vision_to_mission, headers={'id':4})
            rospy.loginfo('connected to mission_srv!')

            self.isStart = True
            isAborted = False

        if req.abort_request:
            isAborted = True

        return mission_to_visionResponse(self.isStart, isAborted)

    def __init__(self):
        smach.State.__init__(
                        self,
                        outcomes=['start_complete', 'killed'],
                        output_keys=['targetIDs']
        )

    def execute(self, userdata):
        global tollbooth
        tollbooth = None

        global firstRun
        if firstRun:
            firstRun = False

            srvServer = rospy.Service('tollbooth_srv', mission_to_vision, self.handle_srv)
            rospy.loginfo('tollbooth_srv initialized!')

        self.isStart = TEST_MODE

        while not self.isStart:
            if rospy.is_shutdown(): return 'killed'
            rosRate.sleep()

        global imageSub, gunSide, currentEye
        imageSub.unregister()
        gunSide = 'left'
        currentEye = 'left'
        imageSub = rospy.Subscriber(imageLeftTopic, Image, gotRosFrame)


        tollbooth = TollboothDetector(params, lock, camdebug)
        tollbooth.heading = cur_heading

        userdata.targetIDs = COMPETITION_TARGETS
        return 'start_complete'


'''
Detect the tollbooth
'''
class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['search_complete', 'aborted', 'killed'],
                             input_keys=['targetIDs'],
                             output_keys=['targetIDs'])

    def execute(self, userdata):
        #HACK: search for single board instead now
        tollbooth.changeTarget('red')
        while not rospy.is_shutdown():
            if tollbooth.regionCount >= 1:
                if not TEST_MODE:
                    mission_srv(search_request=True, task_complete_request=False, task_complete_ctrl=None)
                return 'search_complete'

            if isAborted:
                return 'aborted'

            rosRate.sleep()

        return 'killed'


'''
Performs correction to keep target in centre
'''
class Correction:
    # target can be 'board' or 'hole'
    def __init__(self, target='board', FORWARD_K=12, SIDE_K=10.0, DEPTH_K=1.0, ANGLE_K=0.4,
                 EPSILON_X=0.08, EPSILON_Y=0.08, EPSILON_ANGLE=4,
                 MIN_SIZE=0.033, MAX_SIZE=0.6, timeout=None):

        self.FORWARD_K = -FORWARD_K
        self.SIDE_K = SIDE_K
        self.DEPTH_K = DEPTH_K
        self.ANGLE_K = -ANGLE_K
        self.target = target

        self.EPSILON_X = EPSILON_X
        self.EPSILON_Y = EPSILON_Y
        self.EPSILON_ANGLE = EPSILON_ANGLE
        self.MIN_SIZE = MIN_SIZE
        self.MAX_SIZE = MAX_SIZE

        self.timeout = timeout

        self.hole_offset = (0, -100)

    def correct(self):
        hoverDepth = depth_setpoint
        hoverHeading = tollbooth.heading
        offsets = {}
        offsets['targetLostCount'] = 0

        def tollboothOutOfPlace():
            offsets['x'] = offsets['y'] = 0
            offsets['size'] = offsets['angle'] = 0
            if tollbooth.regionCount == 0:
                offsets['targetLostCount'] += 1
                return True

            if self.target == 'hole' and (not tollbooth.holes or not tollbooth.holes[0]):
                offsets['targetLostCount'] += 1
                return True

            offsets['targetLostCount'] = 0

            x,y,w,h = tollbooth.bigBoundingRect if self.target == 'board' else tollbooth.holes[0][2]
            H,W = tollbooth.shape[0:2]

            if self.target == 'hole':
                x -= self.hole_offset[0]
                y -= self.hole_offset[1]

            sizeRatio = w*h/float(W*H)
            sizeMidPt = (self.MIN_SIZE + self.MAX_SIZE) * 0.5
            if sizeRatio < self.MIN_SIZE or sizeRatio > self.MAX_SIZE:
                offsets['size'] = sizeRatio - sizeMidPt

            rospy.logdebug('(' + self.target + ') vals: ' + str({ 'x': x, 'y': y, 'w': w, 'h': h, 'sizeRatio': sizeRatio }))

            offsets['x'] = clamp((x + w/2)/float(W) - 0.5, -1, 1)
            offsets['y'] = clamp((y + h/2)/float(H) - 0.5, -1, 1)

            if self.target == 'board':
                pts = tollbooth.bigQuad
            else:
                tx,ty,tw,th = tollbooth.holes[0][2]
                pts = [(tx, ty), (tx+tw, ty), (tx+tw, ty+th), (tx, ty+th)]
            angles = [
                math.degrees(math.atan2(pts[1][1]-pts[0][1], pts[1][0]-pts[0][0])),
                math.degrees(math.atan2(pts[2][1]-pts[3][1], pts[2][0]-pts[3][0]))
            ]
            offsets['angle'] = clamp(angles[0] - angles[1], -20, 20)

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

            if not tollboothOutOfPlace():
                lock.release() #HACK
                break

            rospy.logdebug('offsets: ' + str(offsets))

            H,W = tollbooth.shape[0:2]

            goal = None
            waitTime = None

            if abs(offsets['y']) > self.EPSILON_Y:
                print("Correcting vertical")
                x,y,w,h = tollbooth.bigBoundingRect if self.target == 'board' else tollbooth.holes[0][2]

                factor = 1 - h/float(H) # attenuation factor
                hoverDepth = depth_setpoint + factor * self.DEPTH_K * offsets['y']
                goal = bbauv_msgs.msg.ControllerGoal(
                        heading_setpoint = hoverHeading,
                        depth_setpoint = hoverDepth
                )
                waitTime = params['depthGoalWait']
            elif abs(offsets['x']) > self.EPSILON_X:
                print("Correcting horizontal")
                x,y,w,h = tollbooth.bigBoundingRect if self.target == 'board' else tollbooth.holes[0][2]

                factor = 1 - w/float(W)
                goal = bbauv_msgs.msg.ControllerGoal(
                        heading_setpoint = hoverHeading,
                        depth_setpoint = hoverDepth,
                        sidemove_setpoint = factor * self.SIDE_K * offsets['x']
                )
                waitTime = params['otherGoalWait']
            elif offsets['size']:
                print ("Correcting distance")
                goal = bbauv_msgs.msg.ControllerGoal(
                        heading_setpoint = hoverHeading,
                        depth_setpoint = hoverDepth,
                        forward_setpoint = self.FORWARD_K * offsets['size']
                )
                waitTime = params['otherGoalWait']
            elif abs(offsets['angle']) > self.EPSILON_ANGLE:
                print ("Correcting heading")
                hoverHeading = norm_heading(tollbooth.heading + self.ANGLE_K * offsets['angle'])

                goal = bbauv_msgs.msg.ControllerGoal(
                        heading_setpoint = hoverHeading,
                        depth_setpoint = hoverDepth,
                        sidemove_setpoint = math.copysign(3.0, offsets['angle'])
                )
                waitTime = params['otherGoalWait']
            elif offsets['targetLostCount'] > 30:
                #TODO: abort task
                print ("Lost target; hovering")
                actionClient.cancel_all_goals()

            lock.release() #HACK

            if goal is not None:
                actionClient.send_goal(goal)
                actionClient.wait_for_result(rospy.Duration.from_sec(waitTime))
                print("got result")

            rosRate.sleep()


'''
Shift the tollbooth into the centre of the image
'''
class Stabilize(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['found', 'aborted', 'killed'],
                             input_keys=['targetIDs'],
                             output_keys=['targetIDs'])

    def execute(self, userdata):
        initAction()

        correction = Correction(
            FORWARD_K = params['farForwardK'],
            SIDE_K = params['farSideK'],
            DEPTH_K = params['farDepthK'],
            ANGLE_K = params['farAngleK'],
            EPSILON_X = params['farEpsilonX'],
            EPSILON_Y = params['farEpsilonY'],
            EPSILON_ANGLE = params['farEpsilonAngle'],
            MIN_SIZE = params['farMinSize'],
            MAX_SIZE = params['farMaxSize'],
            timeout = rospy.Duration.from_sec(params['farTimeout'])
        )
        result = correction.correct()
        if result in ['aborted', 'killed']:
            return result

        return 'found'


'''
Manoeuvre to target
'''
class MoveToTarget(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['fired', 'fired_all', 'aborted', 'killed'],
                             input_keys=['targetIDs'],
                             output_keys=['targetIDs'])

    def execute(self, userdata):
        targetID = userdata.targetIDs[0]
        remainingIDs = userdata.targetIDs[1:]

        tollbooth.changeTarget(targetID)

        # Adjust to board first
        rospy.loginfo('moving to single colour board')
        correction = Correction(
            target = 'board',
            FORWARD_K = params['nearForwardK'],
            SIDE_K = params['nearSideK'],
            DEPTH_K = params['nearDepthK'],
            ANGLE_K = params['nearAngleK'],
            EPSILON_X = params['nearEpsilonX'],
            EPSILON_Y = params['nearEpsilonY'],
            EPSILON_ANGLE = params['nearEpsilonAngle'],
            MIN_SIZE = params['nearMinSize'],
            MAX_SIZE = params['nearMaxSize'],
            timeout = rospy.Duration.from_sec(params['nearTimeout'])
        )
        correction.correct()

        # Then adjust to hole
        rospy.loginfo('moving to single hole')
        correction = Correction(
            target = 'hole',
            FORWARD_K = params['holeForwardK'],
            SIDE_K = params['holeSideK'],
            DEPTH_K = params['holeDepthK'],
            ANGLE_K = params['holeAngleK'],
            EPSILON_X = params['holeEpsilonX'],
            EPSILON_Y = params['holeEpsilonY'],
            EPSILON_ANGLE = params['holeEpsilonAngle'],
            MIN_SIZE = params['holeMinSize'],
            MAX_SIZE = params['holeMaxSize'],
            timeout = rospy.Duration.from_sec(params['holeTimeout'])
        )
        result = correction.correct()

        if result in ['aborted', 'killed']:
            return result

        # Lock on to target and fire torpedo
        actionClient.cancel_all_goals()

        #rospy.sleep(params['chargeWait'])

        rospy.loginfo("pew pew")
        fireTorpedo(gunSide)

        rospy.sleep(params['fireWait'])

        # If no targets left, continue
        userdata.targetIDs = remainingIDs
        if not remainingIDs:
            return 'fired_all'

        return 'fired'

'''
Backoff until the whole thing is visible again
'''
class Backoff(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['found', 'aborted', 'killed'],
                             input_keys=['targetIDs'],
                             output_keys=['targetIDs'])

    def execute(self, userdata):
        hoverHeading = tollbooth.heading
        hoverDepth = depth_setpoint

        # Switch eye
        global currentEye, imageSub
        if currentEye == 'left' and SWAP_EYE:
            rospy.loginfo('switching to right eye!')
            imageSub.unregister()
            imageSub = rospy.Subscriber(imageRightTopic, Image, gotRosFrame)
            currentEye = 'right'

        global gunSide
        if gunSide == 'left':
            gunSide = 'right'

        if not SINGLE_BOARD_MODE:
            goal = bbauv_msgs.msg.ControllerGoal(
                    heading_setpoint = hoverHeading,
                    depth_setpoint = hoverDepth,
                    forward_setpoint = -7
            )
            actionClient.send_goal(goal)
            actionClient.wait_for_result(rospy.Duration(5.5))

            color = userdata.targetIDs[0]
            tollbooth.changeTarget(color)

            tries = 0

            while not rospy.is_shutdown():
                if isAborted: return 'aborted'

                if tollbooth.regionCount >= 1:
                    return 'found'

                if tries < 2:
                    goal = bbauv_msgs.msg.ControllerGoal(
                            heading_setpoint = hoverHeading,
                            depth_setpoint = hoverDepth,
                            forward_setpoint = -2
                    )
                    actionClient.send_goal(goal)
                    actionClient.wait_for_result(rospy.Duration(1,0))

                    tries += 1
                else:
                    actionClient.cancel_all_goals()

                rosRate.sleep()
        else:
            tollbooth.changeTarget('red')
            return 'found'

        return 'killed'

'''
Carry on to next task
'''
class Done(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                                   input_keys=['headings'])

    def execute(self, userdata):
        if isAborted: return 'aborted'

        # Backoff a little
        goal = bbauv_msgs.msg.ControllerGoal(
                heading_setpoint = cur_heading,
                depth_setpoint = depth_setpoint,
                forward_setpoint = -1
        )
        actionClient.send_goal(goal)
        actionClient.wait_for_result()

        ctrl = controller()
        ctrl.depth_setpoint = depth_setpoint
        ctrl.heading_setpoint = cur_heading
        if not TEST_MODE:
            mission_srv(search_request=False, task_complete_request=True, task_complete_ctrl=ctrl)
        return 'succeeded'


'''
Torpedo functions
- side: 'left' or 'right'
'''
manipulator_pub = None # Publisher
def fireTorpedo(side):
    manip = manipulator()
    manip.servo1 = 0
    manip.servo2 = 0
    manip.servo3 = int(side == 'left')
    manip.servo4 = int(side == 'right')
    manip.servo5 = 0
    manip.servo6 = 0
    manip.servo7 = 0
    manipulator_pub.publish(manip)



'''
Main
'''
if __name__ == '__main__':
    rospy.init_node('tollbooth', anonymous=False)
    loopRateHz = rospy.get_param('~loopHz', 20)
    imageLeftTopic = rospy.get_param('~imageLeft', '/stereo_camera/left/image_rect_color')
    imageRightTopic = rospy.get_param('~imageRight', '/stereo_camera/right/image_rect_color')
    depthTopic = rospy.get_param('~depth', '/depth')
    compassTopic = rospy.get_param('~compass', '/euler')

    manipulator_pub = rospy.Publisher('/manipulators', manipulator)

    # Set up param configuration window
    def configCallback(config, level):
        for param in params:
            params[param] = config[param]
        for param in TollboothDetector.yellow_params:
            TollboothDetector.yellow_params[param] = config['yellow_' + param]
        for param in TollboothDetector.red_params:
            TollboothDetector.red_params[param] = config['red_' + param]
        for param in TollboothDetector.green_params:
            TollboothDetector.green_params[param] = config['green_' + param]
        for param in TollboothDetector.blue_params:
            TollboothDetector.blue_params[param] = config['blue_' + param]
        return config
    srv = Server(TollboothConfig, configCallback)

    camdebug = CamDebug('Vision', debugOn=DEBUG)

    global rosRate
    rosRate = rospy.Rate(loopRateHz)

    def gotRosFrame(rosImage):
        if tollbooth: tollbooth.gotRosFrame(rosImage)
    def gotHeading(msg):
        global cur_heading
        cur_heading = msg.yaw
        if tollbooth: tollbooth.gotHeading(msg)
    def gotDepth(msg):
        global depth_setpoint
        depth_setpoint = msg.depth
    currentEye = 'left'
    imageSub = rospy.Subscriber(imageLeftTopic, Image, gotRosFrame)
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
                        transitions={'found': 'GET_TARGET',
                                     'aborted': 'DISENGAGED',
                                     'killed': 'killed'}
        )
        smach.StateMachine.add(
                        'GET_TARGET',
                        MoveToTarget(),
                        transitions={'fired': 'BACKOFF',
                                     'fired_all': 'DONE',
                                     'aborted': 'DISENGAGED',
                                     'killed': 'killed'}
        )
        smach.StateMachine.add(
                        'BACKOFF',
                        Backoff(),
                        transitions={'found': 'GET_TARGET',
                                     'aborted': 'DISENGAGED',
                                     'killed': 'killed'}
        )
        smach.StateMachine.add(
                        'DONE',
                        Done(),
                        transitions={'succeeded': 'succeeded',
                                     'aborted': 'DISENGAGED'}
        )

    sis = smach_ros.IntrospectionServer('tollbooth_server', sm, '/MISSION/TOLLBOOTH')
    sis.start()

    try:
        outcome = sm.execute()
    except Exception, e:
        print e
        rospy.signal_shutdown('Shutting down due to exception.')

    rospy.signal_shutdown('Completed')

# vim: set sw=4 ts=4 expandtab:
