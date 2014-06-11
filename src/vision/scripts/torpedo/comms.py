#/usr/bin/env python

'''
Communication b/w ROS class and submodules
'''

import roslib; roslib.load_manifest('vision')
import rospy
from front_commons.frontComms import FrontComms
from vision import TorpedoVision

from bbauv_msgs.msg import controller
from bbauv_msgs.srv import mission_to_visionResponse, \
        mission_to_vision, vision_to_mission
        
from dynamic_reconfigure.server import Server as DynServer
#from utils.config import torpedoConfig as Config

class Comms(FrontComms):
    
    isTesting = False
    isKilled = False
    isAborted = False
    isStart = False
    
    # Circle booleans
    foundCircles = False 
    foundSomething = False 
    foundCount = 0  # If more than a constant, then respond lost
    
    # Shooting parameters
    numShoot = 0    # Only given 2 shoots 
    centroidToShoot = None
    
    # Movement parameters
    radius = None
    deltaX = None
    deltaY = None
    deltaXMult = 5.0
    
    def __init__(self):
        FrontComms.__init__(self, TorpedoVision(comms=self))
        self.defaultDepth = 2.0
        
        # Initialise mission planner 
        if not self.isAlone:
            self.comServer = rospy.Service("/torpedo/mission_to_vision", 
                                           mission_to_vision,
                                           self.handle_srv)
            rospy.loginfo("Waiting for mission planner")
            self.toMission = rospy.ServiceProxy("/torpedo/vision_to_mission",
                                                vision_to_mission)
            self.toMission.wait_for_service(timeout=60)
        
    # Handle mission services
    def handle_srv(self, req):
        global isStart
        global isAborted
        global locomotionGoal
        
        rospy.loginfo("Torpedo Service handled")
        
        if req.start_request:
            rospy.loginfo("Torpedo starting")
            self.isStart = True
            self.isAborted = False
            self.defaultDepth = req.start_ctrl.depth_setpoint
            self.inputHeading = req.start_ctrl.heading_setpoint
            
            return mission_to_visionResponse(start_response=True,
                                             abort_response=False,
                                             data=controller(heading_setpoint=
                                                             self.curHeading))
        
        elif req.abort_request:
            rospy.loginfo("Torpedo abort received")
            self.sendMovement(forward=0.0, sidemove=0.0)
            self.isAborted=True
            self.isStart = False
            self.unregister()
            
            return mission_to_visionResponse(start_response=False,
                                             abort_response=True,
                                             data=controller(heading_setpoint=
                                                             self.curHeading))
            
    def shootTopTorpedo(self):
        maniPub = rospy.Publisher("/manipulators", manipulator)
        maniPub.publish(0 | 1)
        rospy.sleep(rospy.Duration(0.2)) 

    def shootBotTorpedo(self):
        maniPub = rospy.Publisher("/manipulators", manipulator)
        maniPub.publish(0 | 2)
        rospy.sleep(rospy.Duration(0.2))       
    
    def reconfigure(self, config, level):
        rospy.loginfo("Received dynamic reconfigure request")
        self.params = {'loThreshold': (config.loH, config.loS, config.loV),
                       'hiThreshold': (config.hiH, config.hiS, config.hiV),
                       'cannyParams': (config.Canny1, config.Canny2),
                       'minContourArea': config.minContourArea }
        self.visionFilter.updateParams()
        return config
    
    def registerSonar(self):
        self.sonarBearing = None
        self.sonarDist = None 
        self.sonarSub = rospy.Subscriber("/sonarData", sonarData, sonarDataCallback)
    
    def sonarDataCallback(self, data):
        self.sonarBearing = data.bearing
        self.sonarDist = data.range
        
    def unregisterSonar(self):
        self.sonarSub.unregister()
    
def main():
    pass