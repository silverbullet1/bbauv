#!/usr/bin/env python2
'''
Code to identify RoboSub tollbooth and carry out task
'''

import roslib; roslib.load_manifest('Vision')
import rospy
from sensor_msgs.msg import Image
import bbauv_msgs
from bbauv_msgs.msg import compass_data, depth
from bbauv_msgs.srv import *

import actionlib
from bbauv_msgs.msg import ControllerAction, controller

import math
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from com.camdebug.camdebug import CamDebug
from com.tollbooth.tollbooth import TollboothDetector
from com.utils.utils import *

from dynamic_reconfigure.server import Server
from Vision.cfg import TollboothConfig

import smach
import smach_ros


#TODO: use actual competition IDs
COMPETITION_TARGETS = ['red', 'yellow']

# GLOBALS
DEBUG = True
camdebug = None
tollbooth = None
firstRunAction = True
depth_setpoint = 0.5
cur_heading = 0
imageSub = None
currentEye = 'left'

# dynamic_reconfigure params; see Tollbooth.cfg
params = { 'contourMinArea': 0,
           'redHueLow': 0, 'redHueHigh': 0, 'redSatLow': 0, 'redSatHigh': 0, 'redValLow': 0, 'redValHigh': 0,
           'blueHueLow': 0, 'blueHueHigh': 0, 'blueSatLow': 0, 'blueSatHigh': 0, 'blueValLow': 0, 'blueValHigh': 0,
           'yellowHueLow': 0, 'yellowHueHigh': 0, 'yellowSatLow': 0, 'yellowSatHigh': 0, 'yellowValLow': 0, 'yellowValHigh': 0,
           'greenHueLow': 0, 'greenHueHigh': 0, 'greenSatLow': 0, 'greenSatHigh': 0, 'greenValLow': 0, 'greenValHigh': 0
}


def clamp(val, minimum, maximum):
    return max(minimum, min(val, maximum))


def initAction():
    global firstRunAction
    if firstRunAction:
        print "waiting for set_controller_srv"
        rospy.wait_for_service('set_controller_srv')
        set_controller_request = rospy.ServiceProxy('set_controller_srv',set_controller)
        rospy.wait_for_service('set_controller_srv')
        set_controller_request(True, True, True, True, False, False, False)
        print "set controller request"

        firstRunAction = False

        global actionClient
        actionClient = actionlib.SimpleActionClient('LocomotionServer', ControllerAction)
        print 'waiting for action server...'
        actionClient.wait_for_server()
        print 'done'


# States
class Disengage(smach.State):
    def handle_srv(self, req):
        global depth_setpoint

        print 'got a request!'
        if req.start_request:
            self.isStart = True
            self.inputHeading = req.start_ctrl.heading_setpoint
            depth_setpoint = req.start_ctrl.depth_setpoint

            rospy.wait_for_service('mission_srv')
            global mission_srv
            mission_srv = rospy.ServiceProxy('mission_srv', vision_to_mission)
            rospy.loginfo('connected to mission_srv!')

        return mission_to_visionResponse(self.isStart, False)

    def __init__(self):
        self.isStart = True #TODO: set to False
        smach.State.__init__(
                        self,
                        outcomes=['start_complete', 'aborted'],
                        output_keys=['targetIDs']
        )

    def execute(self, userdata):
        global tollbooth
        tollbooth = None

        while not self.isStart:
            if rospy.is_shutdown(): return 'aborted'
            rosRate.sleep()

        tollbooth = TollboothDetector(params, camdebug)
        tollbooth.heading = cur_heading

        userdata.targetIDs = COMPETITION_TARGETS
        return 'start_complete'


'''
Detect the tollbooth
'''
class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['search_complete', 'aborted'],
                             input_keys=['targetIDs'],
                             output_keys=['targetIDs'])

    def execute(self, userdata):
        while not rospy.is_shutdown():
            if tollbooth.regionCount >= 3:
#                mission_srv(search_request=True, task_complete_request=False, task_complete_ctrl=None)
                return 'search_complete'
            rosRate.sleep()

        return 'aborted'


'''
Performs correction to keep target in centre
'''
class Correction:
    # target can be 'board' or 'hole'
    def __init__(self, target='board', FORWARD_K=-12, SIDE_K=10.0, DEPTH_K=1.0, ANGLE_K=-0.4,
                 EPSILON_X=0.08, EPSILON_Y=0.08, EPSILON_ANGLE=4,
                 MIN_SIZE=0.033, MAX_SIZE=0.3):

        self.FORWARD_K = FORWARD_K
        self.SIDE_K = SIDE_K
        self.DEPTH_K = DEPTH_K
        self.ANGLE_K = ANGLE_K
        self.target = target

        self.EPSILON_X = EPSILON_X
        self.EPSILON_Y = EPSILON_Y
        self.EPSILON_ANGLE = EPSILON_ANGLE
        self.MIN_SIZE = MIN_SIZE
        self.MAX_SIZE = MAX_SIZE

        self.hole_offset = (0, -100)

    def correct(self):
        hoverDepth = depth_setpoint
        hoverHeading = tollbooth.heading
        offsets = {}

        def tollboothOutOfPlace():
            offsets['x'] = offsets['y'] = 0
            offsets['size'] = offsets['angle'] = 0
            if tollbooth.regionCount == 0:
                return True

            if self.target == 'hole' and (not tollbooth.holes or not tollbooth.holes[0]):
                return True

            x,y,w,h = tollbooth.bigBoundingRect if self.target == 'board' else tollbooth.holes[0][2]
            H,W = tollbooth.shape[0:2]

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

        while tollboothOutOfPlace():
            if rospy.is_shutdown(): return 'aborted'

            print 'offsets:', offsets

            H,W = tollbooth.shape[0:2]

            if abs(offsets['y']) > self.EPSILON_Y:
                print("Correcting vertical")
                x,y,w,h = tollbooth.bigBoundingRect if self.target == 'board' else tollbooth.holes[0][2]

                factor = 1 - h/float(H) # attenuation factor
                hoverDepth = depth_setpoint + factor * self.DEPTH_K * offsets['y']
                goal = bbauv_msgs.msg.ControllerGoal(
                        heading_setpoint = hoverHeading,
                        depth_setpoint = hoverDepth
                )
                actionClient.send_goal(goal)
                actionClient.wait_for_result(rospy.Duration(1,0))
                print("got result")
            elif abs(offsets['x']) > self.EPSILON_X:
                print("Correcting horizontal")
                x,y,w,h = tollbooth.bigBoundingRect if self.target == 'board' else tollbooth.holes[0][2]

                factor = 1 - w/float(W)
                goal = bbauv_msgs.msg.ControllerGoal(
                        heading_setpoint = hoverHeading,
                        depth_setpoint = hoverDepth,
                        sidemove_setpoint = factor * self.SIDE_K * offsets['x']
                )
                actionClient.send_goal(goal)
                actionClient.wait_for_result(rospy.Duration(2,0))
                print("got result")
            elif abs(offsets['angle']) > self.EPSILON_ANGLE:
                print ("Correcting heading")
                hoverHeading = norm_heading(tollbooth.heading + self.ANGLE_K * offsets['angle'])

                goal = bbauv_msgs.msg.ControllerGoal(
                        heading_setpoint = hoverHeading,
                        depth_setpoint = hoverDepth,
                        sidemove_setpoint = math.copysign(3.0, offsets['angle'])
                )
                actionClient.send_goal(goal)
                actionClient.wait_for_result(rospy.Duration(2,0))
                print("got result")
            elif offsets['size']:
                print ("Correcting nearness")
                goal = bbauv_msgs.msg.ControllerGoal(
                        heading_setpoint = hoverHeading,
                        depth_setpoint = hoverDepth,
                        forward_setpoint = self.FORWARD_K * offsets['size']
                )
                actionClient.send_goal(goal)
                actionClient.wait_for_result(rospy.Duration(2,0))
                print("got result")

            rosRate.sleep()


'''
Shift the tollbooth into the centre of the image
'''
class Stabilize(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['found', 'aborted'],
                             input_keys=['targetIDs'],
                             output_keys=['targetIDs'])

    def execute(self, userdata):
        initAction()

        correction = Correction()
        result = correction.correct()
        if result == 'aborted':
            return 'aborted'

        return 'found'


'''
Manoeuvre to target
'''
class MoveToTarget(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['fired', 'fired_all', 'aborted'],
                             input_keys=['targetIDs'],
                             output_keys=['targetIDs'])

    def execute(self, userdata):
        targetID = userdata.targetIDs[0]
        remainingIDs = userdata.targetIDs[1:]

        tollbooth.changeTarget(targetID)

        # Adjust to board first
        rospy.loginfo('moving to single colour board')
        correction = Correction(target='board', FORWARD_K=-2.0, SIDE_K=5.0, DEPTH_K=1.0, ANGLE_K=-0.4, EPSILON_ANGLE=20, MIN_SIZE=0.3, MAX_SIZE=0.8)
        correction.correct()

        # Then adjust to hole
        rospy.loginfo('moving to single hole')
        correction = Correction(target='hole', EPSILON_X=0.04, FORWARD_K=-1.4, SIDE_K=4.0, DEPTH_K=0.3, MIN_SIZE=0.065, MAX_SIZE=0.4)
        result = correction.correct()

        if result == 'aborted':
            return 'aborted'

        #TODO: lock on to target and fire torpedo
        actionClient.cancel_all_goals()

        rospy.sleep(1)

        rospy.loginfo("pew pew")

        rospy.sleep(10)

        #TODO: if no targets left, continue
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
                             outcomes=['found', 'aborted'],
                             input_keys=['targetIDs'],
                             output_keys=['targetIDs'])

    def execute(self, userdata):
        hoverHeading = tollbooth.heading
        hoverDepth = depth_setpoint

#        # Switch eye
#        global currentEye, imageSub
#        if currentEye == 'left':
#            imageSub.unregister()
#            imageSub = rospy.Subscriber(imageRightTopic, Image, gotRosFrame)
#            currentEye = 'right'

        tollbooth.changeTarget('all')

        goal = bbauv_msgs.msg.ControllerGoal(
                heading_setpoint = hoverHeading,
                depth_setpoint = hoverDepth,
                forward_setpoint = -4
        )
        actionClient.send_goal(goal)
        actionClient.wait_for_result(rospy.Duration(2,0))


        while not rospy.is_shutdown():
            if tollbooth.regionCount >= 3:
                return 'found'

            goal = bbauv_msgs.msg.ControllerGoal(
                    heading_setpoint = hoverHeading,
                    depth_setpoint = hoverDepth,
                    forward_setpoint = -2
            )
            actionClient.send_goal(goal)
            actionClient.wait_for_result(rospy.Duration(2,0))

            rosRate.sleep()
        return 'aborted'

'''
Carry on to next task
'''
class Done(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                                   input_keys=['headings'])

    def execute(self, userdata):
        #TODO: move to a spot to find lane marker
        ctrl = controller()
        ctrl.depth_setpoint = depth_setpoint
        ctrl.heading_setpoint = cur_heading
#        mission_srv(search_request=False, task_complete_request=True, task_complete_ctrl=ctrl)
        return 'succeeded'



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

    # Set up param configuration window
    def configCallback(config, level):
        for param in params:
            params[param] = config[param]
        return config
    srv = Server(TollboothConfig, configCallback)

    camdebug = CamDebug('tollbooth', debugOn=DEBUG)

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

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
    with sm:
        smach.StateMachine.add(
                        'DISENGAGED',
                        Disengage(),
                        transitions={'start_complete':'SEARCH',
                                     'aborted': 'aborted'}
        )
        smach.StateMachine.add(
                        'SEARCH',
                        Search(),
                        transitions={'search_complete':'STABILIZE',
                                     'aborted': 'aborted'}
        )
        smach.StateMachine.add(
                        'STABILIZE',
                        Stabilize(),
                        transitions={'found': 'GET_TARGET',
                                     'aborted': 'aborted'}
        )
        smach.StateMachine.add(
                        'GET_TARGET',
                        MoveToTarget(),
                        transitions={'fired': 'BACKOFF',
                                     'fired_all': 'DONE',
                                     'aborted': 'aborted'}
        )
        smach.StateMachine.add(
                        'BACKOFF',
                        Backoff(),
                        transitions={'found': 'GET_TARGET',
                                     'aborted': 'aborted'}
        )
        smach.StateMachine.add(
                        'DONE',
                        Done(),
                        transitions={'succeeded': 'succeeded',
                                     'aborted': 'aborted'}
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
