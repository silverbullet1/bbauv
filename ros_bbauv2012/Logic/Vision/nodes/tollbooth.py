#!/usr/bin/env python2
'''
Code to identify RoboSub tollbooth and carry out task
'''

import roslib; roslib.load_manifest('Vision')
import rospy
from sensor_msgs.msg import Image
from bbauv_msgs.msg import compass_data

import math
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from com.camdebug.camdebug import CamDebug
from com.tollbooth.tollbooth import TollboothDetector

from dynamic_reconfigure.server import Server
from Vision.cfg import TollboothConfig

import smach


DEBUG = True
camdebug = None
laneDetector = None

# dynamic_reconfigure params; see Tollbooth.cfg
params = { 'hueLow1': 0, 'hueHigh1': 0, 'satLow1': 0, 'satHigh1': 0, 'valLow1': 0, 'valHigh1': 0,
           'hueLow2': 0, 'hueHigh2': 0, 'satLow2': 0, 'satHigh2': 0, 'valLow2': 0, 'valHigh2': 0,
           'hueLow3': 0, 'hueHigh3': 0, 'satLow3': 0, 'satHigh3': 0, 'valLow3': 0, 'valHigh3': 0,
           'hueLow4': 0, 'hueHigh4': 0, 'satLow4': 0, 'satHigh4': 0, 'valLow4': 0, 'valHigh4': 0
}


# States
'''
Search for the tollbooth and centre it in sights
'''
class FindTollboothState(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['found', 'aborted'],
                             input_keys=['targetColours'],
                             output_keys=['targetColours'])

    def execute(self, userdata):
        #TODO: find the tollbooth
        while not rospy.is_shutdown():
            rosRate.sleep()
        return 'found'

'''
Manoeuvre to target
'''
class MoveToTargetState(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['done', 'fired', 'aborted'],
                             input_keys=['targetColours'],
                             output_keys=['targetColours'])

    def execute(self, userdata):
        #TODO: lock on to target and fire torpedo
        #TODO: if no targets left, continue
        return 'done'

'''
Carry on to next task
'''
class FoundState(smach.State):
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
    imageTopic = rospy.get_param('~image', '/stereo_camera/left/image_color')
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

    global tollbooth
    tollbooth = TollboothDetector(params, camdebug)

    def gotRosFrame(rosImage): tollbooth.gotRosFrame(rosImage)
    def gotHeading(msg): tollbooth.gotHeading(msg)
    rospy.Subscriber(imageTopic, Image, gotRosFrame)
    rospy.Subscriber(compassTopic, compass_data, gotHeading)

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
    with sm:
        smach.StateMachine.add('FindTollboothState', FindTollboothState(),
                                transitions={'found': 'MoveToTargetState',
                                             'aborted': 'aborted'})
        smach.StateMachine.add('MoveToTargetState', MoveToTargetState(),
                                transitions={'done': 'FoundState',
                                             'fired': 'MoveToTargetState',
                                             'aborted': 'aborted'})
        smach.StateMachine.add('FoundState', FoundState(),
                                transitions={'succeeded': 'succeeded',
                                             'aborted': 'aborted'})

    outcome = sm.execute()

# vim: set sw=4 ts=4 expandtab:
