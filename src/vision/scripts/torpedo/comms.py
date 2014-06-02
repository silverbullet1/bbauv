#/usr/bin/env python

'''
Communication b/w ROS class and submodules
'''

import rospy
from front_commons.frontComms import FrontComms
from vision import TorpedoVision

from bbauv_msgs.msg import controller
from bbauv_msgs.srv import mission_to_visionResponse, \
        mission_to_vision, vision_to_mission

class Comms(FrontComms):
    
    isTesting = False
    isKilled = False
    isAborted = False
    isStart = False
    
    # Circle booleans
    foundCircles = False 
    
    # Shooting parameters
    numShoot = 0    # Only given 2 shoots 
    centroidToShoot = None
    medianCentroid = []
    medianRadius = []
    
    # Movement parameters
    radius = None
    deltaX = None
    deltaXMult = 5.0
    
    def __init__(self):
        FrontComms.__init__(self, TorpedoVision(comms=self))
        self.defaultDepth = 0.6
        
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
            isStart = True
            isAborted = False
            self.defaultDepth = req.start_ctrl.depth_setpoint
            self.inputHeading = req.start_ctrl.heading_setpoint
            
            return mission_to_visionResponse(start_response=True,
                                             abort_response=False,
                                             data=controller(heading_setpoing=
                                                             self.curHeading))
        
        elif req.abort_request:
            rospy.loginfo("Torpedo abort received")
            self.sendMovement(forward=0.0, sidemove=0.0)
            isAbort=True
            isStart = False
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
    
    
def main():
    pass