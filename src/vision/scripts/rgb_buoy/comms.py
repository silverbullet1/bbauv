#!/usr/bin/env python

'''
Communication b/w ROS class and submodules
'''

import rospy

from front_commons.frontComms import FrontComms
from vision import RgbBuoyVision

class Comms():

    isTesting = False
    isKilled = False
    isAborted = False

    def __init__(self):
        FrontComms = FrontComms.__init__(self, RgbBuoyVision(comms=self))

    #Handle mission services
    def handle_srv(self, req):
        global isStart
        global isAborted
        global locomotionGoal
        global rgb_buoy

        rospy.loginfo("RGB Service handled")

        if req.start_request:
            rospy.loginfo("RGB starting")
            isStart = True
            isAborted = False

        if req.abort_request:
            rospy.loginfo("Flare abort received")
            isAbort=True
            isStart = False
            FrontComms.unregister()

        return mission_to_visionResponse(isStart, isAborted)

def main():
    testCom = Comms()
