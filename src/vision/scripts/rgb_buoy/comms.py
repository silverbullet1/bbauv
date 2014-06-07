#/usr/bin/env python

'''
Communication b/w ROS class and submodules
'''

import rospy

from front_commons.frontComms import FrontComms
from vision import RgbBuoyVision

from dynamic_reconfigure.server import Server as DynServer
#from utils.config import rgbConfig as Config
from bbauv_msgs.srv._vision_to_mission import vision_to_mission
from bbauv_msgs.srv._mission_to_vision import mission_to_visionResponse

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
        
        self.dynServer = DynServer(Config, self.reconfigure)
        
        if not self.isAlone:
            #Initialise mission planner
            rospy.loginfo("Starting /rgb/mission_to_vision")
            self.comServer = rospy.Service("/rgb/mission_to_vision",
                                           mission_to_vission, self.handle_srv)
            rospy.loginfo("Waiting for vision to mission service")
            self.toMission = rospy.ServiceProxy("/rgb/vision_to_mission",
                                                vision_to_mission)
            self.toMission.wait_for_service(timeout=60)
        
    # Handle mission services
    def handle_srv(self, req):
        global isStart
        global isAborted
        global locomotionGoal

        rospy.loginfo("RGB Service handled")

        if req.start_request:
            rospy.loginfo("RGB starting")
            self.isStart = True
            self.isAborted = False
            self.defaultDepth = req.start_ctrl.depth_setpoint
            self.inputHeading = req.start_ctrl.heading_setpoint
            
            from bbauv_msgs.msg._controller import controller
            return mission_to_visionResponse(start_response=True,
                                             abort_response=False,
                                             data=controller(heading_setpoint=self.curHeading))

        if req.abort_request:
            rospy.loginfo("RGB abort received")
            self.isAborted=True
            self.isStart = False
            self.sendMovement(forward=0.0, sidemove=0.0)
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
