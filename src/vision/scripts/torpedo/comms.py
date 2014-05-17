#/usr/bin/env python

'''
Communication b/w ROS class and submodules
'''

import rospy

from front_commons.frontComms import FrontComms
from vision import TorpedoVision

class Comms(FrontComms):
    
    isTesting = False
    isKilled = False
    isAborted = False
    isStart = False
    
    # Vision boolean
    foundCircle = True 
    numShoot = 0    # Only given 2 shoots 
    
    def __init__(self):
        FrontComms.__init__(self, TorpedoVision(comms=self))
        
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
        
        if req.abort_request:
            rospy.loginfo("Torpedo abort received")
            isAbort=True
            isStart = False
            self.unregister()
            
        return mission_to_visionResponse(isStart, isAborted)

def main():
    testCom = Comms()