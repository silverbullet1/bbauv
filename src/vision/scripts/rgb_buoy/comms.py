#!/usr/bin/env python

'''
Communication b/w ROS class and submodules
'''

import rospy

from front_commons.frontComms import FrontComms
from vision import RgbBuoyVision

class Comms(FrontComms):
    
    isTesting = False
    isKilled = False
    isAborted = False
    isStart = False
    
    # Vision boolean
    toBump = False
    foundBuoy = False
    centroidToBump = None
    colourToBump = "RED"
    
    def __init__(self):
        FrontComms.__init__(self, RgbBuoyVision(comms=self))
        
    # Handle mission services
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
            rospy.loginfo("RGB abort received")
            isAbort=True
            isStart = False
            self.unregister()
            
        return mission_to_visionResponse(isStart, isAborted)

def main():
    testCom = Comms()
