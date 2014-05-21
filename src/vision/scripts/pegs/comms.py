#/usr/bin/env/python 

'''
For communication with Robot 
'''

import rospy
from front_commons.frontComms import FrontComms
from vision import PegsVision

class Comms(FrontComms):
    
    isTesting = False
    isKilled = False 
    isAborted = False 
    isStart = False
    
    # Vision parameters 
    findYellowBoard = True  # First find yellow board
    foundYellowBoard = False 
    yellowCoordinates = (-1, -1)
    
    timeToFindPegs = False 
    findRedPeg = True   #Either find red or find blue circle 
    foundSomething = False 
    
    count = 0       # Move up to 4 pegs 
    
    centroidToPick = (-1, -1)
    deltaX = None
    deltaXMult = 5.0
    areaRect = 0
    
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

    def navigationRegister(self):
        self.earth_odom_sub = rospy.Subscriber('/earth_odom', Odometry, self.earthOdomCallback)

    def navigationUnregister(self):
        self.earth_odom_sub.unregister()
    
    def earthOdomCallback(self, data):
        self.yellowCoordinates = (data.pose.pose.position.x, data.pose.pose.position.y)
        self.navigationUnregister()
        rospy.loginfo("Current coordinate of yellow board is: ({},{})".format(self.yellowCoordinates[0], 
                                                                             self.yellowCoordinates[1]))
    
    def goToPos(self):
        handle = rospy.ServiceProxy('/navigate2D', navigate2d)
        handle(x=self.yellowCoordinates[0], y=self.yellowCoordinates[1])
        rospy.loginfo("Moving to the center of yellow board")

def main():
    testCom = Comms()
        
