#/usr/bin/env python

'''
Communication b/w ROS class and submodules
'''

import rospy

from front_commons.frontComms import FrontComms
from vision import RgbBuoyVision

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
        #self.colourToBump = int(rospy.get_param("~color", "0"))
        
    # Handle mission services
    def handle_srv(self, req):
        global isStart
        global isAborted
        global locomotionGoal

        rospy.loginfo("RGB Service handled")

        if req.start_request:
            rospy.loginfo("RGB starting")
            isStart = True
            isAborted = False

        if req.abort_request:
            rospy.loginfo("RGB abort received")
            isAbort=True
            isStart = False
            self.unregister()
            
        return mission_to_visionResponse(isStart, isAborted)

    def navigationRegister(self):
        self.earth_odom_sub = rospy.Subscriber('/earth_odom', Odometry, self.earthOdomCallback)

    def navigationUnregister(self):
        self.earth_odom_sub.unregister()
    
    def earthOdomCallback(self, data):
        self.rgbCoordinates = (data.pose.pose.position.x, data.pose.pose.position.y)
        self.navigationUnregister()
        rospy.loginfo("Current coordinate of rgb buoy is: ({},{})".format(self.rgbCoordinates[0], 
                                                                             self.rgbCoordinates[1]))
    
    def goToPos(self):
        handle = rospy.ServiceProxy('/navigate2D', navigate2d)
        handle(x=self.rgbCoordinates[0], y=self.rgbCoordinates[1])
        rospy.loginfo("Moving to the center of rgb buoy")

def main():
    testCom = Comms()
