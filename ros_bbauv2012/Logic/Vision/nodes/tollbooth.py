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


DEBUG = True
camdebug = None
tollbooth = None
firstRunAction = True
depth_setpoint = 0.5
cur_heading = 0

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

        #TODO: use actual competition IDs
        userdata.targetIDs = ['red', 'blue']
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
                return 'search_complete'
            rosRate.sleep()
        return 'aborted'

class Correction:
    # target can be 'board' or 'hole'
    def __init__(self, target='board', FORWARD_K=-1.0, SIDE_K=4.0, DEPTH_K=4.0, ANGLE_K=-0.2,
                 EPSILON_X=0.08, EPSILON_Y=0.08, EPSILON_SIZE=0.2, EPSILON_ANGLE=5):
        self.FORWARD_K = FORWARD_K
        self.SIDE_K = SIDE_K
        self.DEPTH_K = DEPTH_K
        self.ANGLE_K = ANGLE_K
        self.target = target

        self.EPSILON_X = EPSILON_X
        self.EPSILON_Y = EPSILON_Y
        self.EPSILON_SIZE = EPSILON_SIZE
        self.EPSILON_ANGLE = EPSILON_ANGLE

    def correct(self):
        hoverDepth = depth_setpoint
        hoverHeading = tollbooth.heading
        offsets = {}

        def tollboothOutOfPlace():
            if tollbooth.regionCount == 0:
                offsets['x'] = offsets['y'] = 0
                offsets['size'] = offsets['angle'] = 0
                return True

            if self.target == 'board' and (not tollbooth.holes or not tollbooth.holes[0]):
                return True

            x,y,w,h = tollbooth.bigBoundingRect if self.target == 'board' else tollbooth.holes[0][2]
            H,W = tollbooth.shape[0:2]
            offsets['x'] = clamp((x + w/2)/float(W) - 0.5, -1, 1)
            offsets['y'] = clamp((y + h/2)/float(H) - 0.5, -1, 1)
            offsets['size'] = min(w*h/float(H*W) - 0.7, 1)

            pts = tollbooth.bigQuad if self.target == 'board' else tollbooth.holes[0][2]
            angles = [
                math.degrees(math.atan2(pts[1][1]-pts[0][1], pts[1][0]-pts[0][0])),
                math.degrees(math.atan2(pts[2][1]-pts[3][1], pts[2][0]-pts[3][0]))
            ]
            offsets['angle'] = clamp(angles[0] - angles[1], -180, 180)

            if abs(offsets['x']) > self.EPSILON_X or abs(offsets['y']) > self.EPSILON_Y:
                return True
            if abs(offsets['size']) > self.EPSILON_SIZE or abs(offsets['angle']) > self.EPSILON_ANGLE:
                return True

            return False

        while tollboothOutOfPlace():
            if rospy.is_shutdown(): return 'aborted'

            x,y,w,h = tollbooth.bigBoundingRect
            H,W = tollbooth.shape[0:2]

            if abs(offsets['y']) > self.EPSILON_Y:
                print("Correcting vertical")
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
                        depth_setpoint = hoverDepth
                )
                actionClient.send_goal(goal)
                actionClient.wait_for_result(rospy.Duration(2,0))
                print("got result")
            elif abs(offsets['size']) > self.EPSILON_SIZE:
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
        correction.correct()

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
        correction = Correction(target='board', FORWARD_K=-1.0, SIDE_K=4.0, DEPTH_K=2.0, ANGLE_K=-0.2)
        correction.correct()

        # Then adjust to hole
        correction = Correction(target='hole', FORWARD_K=-1.0, SIDE_K=4.0, DEPTH_K=2.0, ANGLE_K=-0.2)
        correction.correct()

        #TODO: lock on to target and fire torpedo
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

        tollbooth.changeTarget('all')

        while not rospy.is_shutdown():
            if tollbooth.regionCount >= 3:
                return 'found'

            goal = bbauv_msgs.msg.ControllerGoal(
                    heading_setpoint = hoverHeading,
                    depth_setpoint = hoverDepth,
                    forward_setpoint = -2
            )
            actionClient.send_goal(goal)
            actionClient.wait_for_result(rospy.Duration(1,0))

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
        return 'succeeded'



'''
Main
'''
if __name__ == '__main__':
    rospy.init_node('tollbooth', anonymous=False)
    loopRateHz = rospy.get_param('~loopHz', 20)
    imageTopic = rospy.get_param('~image', '/stereo_camera/left/image_rect_color')
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
    rospy.Subscriber(imageTopic, Image, gotRosFrame)
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
