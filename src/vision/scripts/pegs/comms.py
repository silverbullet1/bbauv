#/usr/bin/env/python 

'''
For communication with Robot 
'''

import rospy
from front_commons.frontComms import FrontComms
from vision import PegsVision

class Comms(FrontComms):
      
    # Vision parameters 
    foundRedPeg = False
    findRedPeg = False   #Either find red or find blue circle 
    count = 0       # Move up to 4 pegs 
    centroidToPick = [-1, -1]
    
    findYellowSquare = True
    foundYellowSquare = False  
    yellowCentroid = [-1, -1]
    yellowArea = -1
    
    def __init__(self):
        FrontComms.__init__(self, PegsVision(comms=self))
        
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

    def foundSomething(self):
        if foundYellowSquare or foundRedPeg:
            return True
        return False
        
def main():
    testCom = Comms()
        