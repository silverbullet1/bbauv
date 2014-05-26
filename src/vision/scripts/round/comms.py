#/usr/bin/env/python 

'''
For communication with Robot 
'''

import rospy
from front_commons.frontComms import FrontComms
from vision import RoundVision

class Comms(FrontComms):

    isTesting = False
    isKilled = False
    isAborted = False
    isStart = False   
    
    # Vision boolean
    centerCentroid = (-1, -1)
    foundSomething = False 
    
    foundRed = False
    foundGreen = False 
    
    # Movement parameters
    firstCross = True   # First cross, then turn 90 deg, then second cross 
    areaRect = None
    deltaX = None
    deltaXMult = 5.0
    
    def __init__(self):
        FrontComms.__init__(self, RoundVision(comms=self))
        
    # Handle mission services
    def handle_srv(self, req):
        global isStart
        global isAborted
        global locomotionGoal
        
        rospy.loginfo("Round Service handled")
        
        if req.start_request:
            rospy.loginfo("Round starting")
            isStart = True
            isAborted = False
        
        if req.abort_request:
            rospy.loginfo("Round abort received")
            isAbort=True
            isStart = False
            self.unregister()
            
        return mission_to_visionResponse(isStart, isAborted)

def main():
    testCom = Comms()        
    
    