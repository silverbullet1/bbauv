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

from dynamic_reconfigure.server import Server
from Vision.cfg import TollboothConfig

import smach
import smach_ros


DEBUG = True
camdebug = None
tollbooth = None
firstRunAction = True
depth_setpoint = 0.5

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
        set_controller_request(True, True, True, True, False, False)
        print "set controller request"

        firstRunAction = False

        global actionClient
        actionClient = actionlib.SimpleActionClient('LocomotionServer', ControllerAction)
        print 'waiting for action server...'
        actionClient.wait_for_server()
        print 'done'


# States
'''
Search for the tollbooth and centre it in sights
'''
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
                        output_keys=['targetColours']
        )

    def execute(self, userdata):
        global tollbooth
        tollbooth = None

        while not self.isStart:
            if rospy.is_shutdown(): return 'aborted'
            rosRate.sleep()

        tollbooth = TollboothDetector(params, camdebug)
        return 'start_complete'


'''
Detect the tollbooth
'''
class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['search_complete', 'aborted'],
                             input_keys=['targetColours'],
                             output_keys=['targetColours'])

    def execute(self, userdata):
        #TODO: find the tollbooth
        while tollbooth.regionCount < 3:
            if rospy.is_shutdown(): return 'aborted'
            rosRate.sleep()

        return 'search_complete'

'''
Shift the tollbooth into the centre of the image
'''
class Stabilize(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['found', 'aborted'],
                             input_keys=['targetColours'],
                             output_keys=['targetColours'])

    def execute(self, userdata):
        EPSILON_X, EPSILON_Y = 0.08, 0.08
        EPSILON_SIZE = 0.08
        EPSILON_ANGLE = 4

        hoverDepth = depth_setpoint
        hoverHeading = tollbooth.heading
        offsets = {}

        initAction()

        def tollboothOutOfPlace():
            x,y,w,h = tollbooth.bigBoundingRect
            H,W = tollbooth.shape[0:2]
            offsets['x'] = clamp((x + w/2)/float(W) - 0.5, -1, 1)
            offsets['y'] = clamp((y + h/2)/float(H) - 0.5, -1, 1)
            offsets['size'] = min(max(w/float(W) - 0.95, h/float(H) - 0.95), 1)

            pts = tollbooth.bigQuad
            angles = [
                math.degrees(math.atan2(pts[1][1]-pts[0][1], pts[1][0]-pts[0][0])),
                math.degrees(math.atan2(pts[2][1]-pts[3][1], pts[2][0]-pts[3][0]))
            ]
            offsets['angle'] = clamp(angles[0] - angles[1], -180, 180)

            if abs(offsets['x']) > EPSILON_X or abs(offsets['y']) > EPSILON_Y:
                return True
            if abs(offsets['size']) > EPSILON_SIZE or abs(offsets['angle']) > EPSILON_ANGLE:
                return True

            return True #TODO: set to False

        while tollboothOutOfPlace():
            if rospy.is_shutdown(): return 'aborted'

            print offsets
            if abs(offsets['y']) > EPSILON_Y:
                print("Correcting vertical")
                hoverDepth = depth_setpoint + 2.0*offsets['y']
                goal = bbauv_msgs.msg.ControllerGoal(
                        heading_setpoint = hoverHeading,
                        depth_setpoint = hoverDepth
                )
                print("send goal")
                actionClient.send_goal(goal)
                actionClient.wait_for_result(rospy.Duration(4,0))
                print("got result")
            elif abs(offsets['x']) > EPSILON_X:
                print("Correcting horizontal")
                goal = bbauv_msgs.msg.ControllerGoal(
                        heading_setpoint = hoverHeading,
                        depth_setpoint = hoverDepth,
                        sidemove_setpoint = 2.0 * offsets['x']
                )
                print("send goal")
                actionClient.send_goal(goal)
                actionClient.wait_for_result(rospy.Duration(4,0))
                print("got result")
            elif abs(offsets['angle']) > EPSILON_ANGLE:
                print ("Correcting heading")
                hoverHeading = tollbooth.heading + 0.1*offsets['angle']

                goal = bbauv_msgs.msg.ControllerGoal(
                        heading_setpoint = hoverHeading,
                        depth_setpoint = hoverDepth
                )
                print("send goal")
                actionClient.send_goal(goal)
                actionClient.wait_for_result(rospy.Duration(4,0))
                print("got result")
            elif abs(offsets['size']) > EPSILON_SIZE:
                print ("Correcting nearness")
                goal = bbauv_msgs.msg.ControllerGoal(
                        heading_setpoint = hoverHeading,
                        depth_setpoint = hoverDepth,
                        forward_setpoint = -1.0 * offsets['size']
                )
                print("send goal")
                actionClient.send_goal(goal)
                actionClient.wait_for_result(rospy.Duration(4,0))
                print("got result")
            else:
                print("nothing to do")

            rosRate.sleep()

        return 'found'

'''
Manoeuvre to target
'''
class MoveToTarget(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['fired', 'aborted'],
                             input_keys=['targetColours'],
                             output_keys=['targetColours'])

    def execute(self, userdata):
        #TODO: lock on to target and fire torpedo
        #TODO: if no targets left, continue
        return 'fired'

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
                        transitions={'found': 'GET_TARGET1',
                                     'aborted': 'aborted'}
        )
        smach.StateMachine.add(
                        'GET_TARGET1',
                        MoveToTarget(),
                        transitions={'fired': 'GET_TARGET2',
                                     'aborted': 'aborted'}
        )
        smach.StateMachine.add(
                        'GET_TARGET2',
                        MoveToTarget(),
                        transitions={'fired': 'DONE',
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
