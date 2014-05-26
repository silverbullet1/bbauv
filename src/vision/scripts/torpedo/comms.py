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
    
    # Circle booleans
    foundCircles = False 
    
    # Shooting parameters
    numShoot = 0    # Only given 2 shoots 
    centroidToShoot = None
    
    # Movement parameters
    radius = None
    deltaX = None
    deltaXMult = 5.0
    
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

    def shootTopTorpedo(self):
        maniPub = rospy.Publisher("/manipulators", manipulator)
        maniPub.publish(0 | 1)
        rospy.sleep(rospy.Duration(0.2)) 

    def shootBotTorpedo(self):
        maniPub = rospy.Publisher("/manipulators", manipulator)
        maniPub.publish(0 | 2)
        rospy.sleep(rospy.Duration(0.2))       
    
    
def main():
    testCom = Comms()