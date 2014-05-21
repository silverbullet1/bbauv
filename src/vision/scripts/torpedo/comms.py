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
    
    # Vision booleans
    findGreenBoard = True   # First find green board
    foundGreenBoard = False
    greenCentroid = (-1, -1)
    greenArea = -1
    
    # Green board position
    greenCoordinates = (-1, -1)
    
    # Circle booleans
    timeToFindCircles = False
    foundCircles = False 
    
    # Shooting parameters
    numShoot = 0    # Only given 2 shoots 
    centroidToShoot = (-1, -1)
    areaRect = 0    
    
    # Movement parameters
    deltaX = 100
    greenPos = (0, 0)
    
    def __init__(self):
        FrontComms.__init__(self, TorpedoVision(comms=self))
        self.earth_odom_sub = None
        
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

    def navigationRegister(self):
        self.earth_odom_sub = rospy.Subscriber('/earth_odom', Odometry, self.earthOdomCallback)

    def navigationUnregister(self):
        self.earth_odom_sub.unregister()
    
    def earthOdomCallback(self, data):
        greenCoordinates = (data.pose.pose.position.x, data.pose.pose.position.y)
        self.navigationUnregister()
    
def main():
    testCom = Comms()