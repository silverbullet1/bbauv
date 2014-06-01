#/usr/bin/env/python 

'''
For communication with Robot 
'''

import rospy
from front_commons.frontComms import FrontComms
from vision import PegsVision

from bbauv_msgs.msg._manipulator import manipulator
from bbauv_msgs.msg import controller
from bbauv_msgs.srv import mission_to_visionResponse, \
        mission_to_vision, vision_to_mission

class Comms(FrontComms):
    
    isTesting = False
    isKilled = False 
    isAborted = False 
    isStart = False
    
    # Vision parameters     
    findRedPeg = True   #Either find red or find blue circle 
    foundSomething = False 
    
    count = 0       # Move up to 4 pegs 
    
    centroidToPick = None
    deltaX = 0
    areaRect = 0
    centering = False 
    
    def __init__(self):
        FrontComms.__init__(self, PegsVision(comms=self))
        self.defaultDepth = 0.6
        
        # Initialise mission planner 
        if not self.isAlone:
            self.comServer = rospy.Service("/pegs/mission_to_vision", 
                                           mission_to_vision,
                                           self.handle_srv)
            rospy.loginfo("Waiting for mission planner")
            self.toMission = rospy.ServiceProxy("/pegs/vision_to_mission",
                                                vision_to_mission)
            self.toMission.wait_for_service(timeout=60)
        
    # Handle mission services
    def handle_srv(self, req):
        global isStart
        global isAborted
        global locomotionGoal
                
        rospy.loginfo("Pegs Service handled")
        
        if req.start_request:
            rospy.loginfo("Pegs starting")
            isStart = True
            isAborted = False
        
        if req.abort_request:
            rospy.loginfo("Pegs abort received")
            isAbort=True
            isStart = False
            self.unregister()
            
        return mission_to_visionResponse(isStart, isAborted)

    def grabRedPeg(self):
        maniPub = rospy.Publisher("/manipulators", manipulator)
        maniPub.publish(0 | 4)
        rospy.sleep(rospy.Duration(0.2))
        
    def putPeg(self):
        maniPub = rospy.Publisher("/manipulators", manipulator)
        maniPub.publish(1 & 4)
        rospy.sleep(rospy.Duration(0.2))

def main():
    testCom = Comms()
        
