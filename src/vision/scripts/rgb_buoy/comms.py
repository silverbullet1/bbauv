#/usr/bin/env python

'''
Communication b/w ROS class and submodules
'''

import rospy

from front_commons.frontComms import FrontComms
from vision import RgbBuoyVision
from apport_python_hook import CONFIG

class Comms(FrontComms):
    
    isTesting = False
    isKilled = False
    isAborted = False
    isStart = False
    
    # Vision boolean
    toBumpColor = False
    foundBuoy = False
    centroidToBump = (-1,-1)
    rectArea = None
    deltaX = 0
    
    isCentering = False 
    
    def __init__(self):
        FrontComms.__init__(self, RgbBuoyVision(comms=self))
        #self.defaultDepth = 1.5
        self.defaultDepth = 2.25
        #self.colourToBump = int(rospy.get_param("~color", "0"))
        
    # Handle mission services
    def handle_srv(self, req):
        global isStart
        global isAborted
        global locomotionGoal

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
    
    def reconfigure(self, config, level):
        rospy.loginfo("Received dynamic reconfigure request")
        self.params = {'hsvLoThres': (config.loH, config.loS, config.loV),
                       'hsvHiThres': (config.hiH, config.hiS, config.hiV),
                       'HoughParams': (config.Hough1, config.Hough2), 
                       'minContourArea' : config.contourMinArea}
        self.visionFilter.updateParams()
        return config

def main():
    testCom = Comms()
