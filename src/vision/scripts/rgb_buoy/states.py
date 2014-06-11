#!/usr/bin/env python

'''
Buoy states
'''

import roslib; roslib.load_manifest('vision')
import rospy

import time
import smach, smach_ros

from comms import Comms

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from vision import RgbBuoyVision

from dynamic_reconfigure.server import Server
from front_commons.frontCommsVision import FrontCommsVision as vision

#Globals
locomotionGoal = None
toBangColour = False

class Disengage(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['start_complete', 'killed'])
        self.comms = comms
    
    def execute(self, userdata):
        while self.comms.isAborted:
            if self.comms.isKilled:
                return 'killed'
            rospy.sleep(rospy.Duration(0.3))
        
        if self.comms.isAlone:
            self.comms.register()
            rospy.sleep(rospy.Duration(0.8))
            self.comms.inputHeading = self.comms.curHeading
            rospy.loginfo("Starting RGB")
            
        self.comms.sendMovement(depth=self.comms.defaultDepth,
                                heading=self.comms.inputHeading,
                                blocking=True)
        
        return 'start_complete'
    
class Search(smach.State):
    timeout = 1000
    moveOnce = 0
    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['search_complete', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, ud):
        start = time.time()
        while not self.comms.foundBuoy:
            if self.comms.isKilled:
                return 'killed'
            if self.comms.isAborted or (time.time() - start) > self.timeout:
                self.comms.isAborted = True
                return 'aborted' 
            
            # Search pattern
            self.comms.sendMovement(forward=0.2)
#             if self.moveOnce < 10:
#                 self.comms.sendMovement(forward=0.2, sidemove=0.2, blocking=False)
#             else:
#                 self.comms.sendMovement(forward=0.2, sidemove=-0.2, blocking=False)        
            
            rospy.sleep(rospy.Duration(0.3))
        
        return 'search_complete'
        
# Precise movements when near buoy 
class Centering (smach.State):
    deltaXMult = 3.0
    deltaYMult = 0.2
    depthCount = 0
    count = 0
    depthCorrected = False 
    
    bigArea = 15000
    changeMultArea = 10000
    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['centering', 'centering_complete', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, userdata):
        self.comms.isCentering = True

        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted'
        
        rospy.loginfo("Delta X: {}".format(self.comms.deltaX))
        rospy.loginfo("Delta Y: {}".format(self.comms.deltaY))

        if self.comms.rectArea > self.changeMultArea:
            self.deltaXMult = 1.5

#         if self.count > 50:
#             rospy.loginfo("Banging")
#             self.comms.sendMovement(forward=2.0, tblocking=False)   # Shoot forward

        if self.comms.rectArea > self.bigArea:
            self.comms.sendMovement(forward=2.5, timeout=1.5, blocking=False)   # Shoot forward
            self.comms.sendMovement(forward=-1.5, timeout=1.5, blocking=False)  # Reverse a bit
            self.comms.isAborted = True
            self.comms.isKilled = True 
            return 'centering_complete'
        
#         if abs(self.comms.deltaX) < 0.005 and abs(self.comms.deltaY) < 0.005:
#             self.count = self.count + 1
#             rospy.loginfo("Count: {}".format(self.count))

#         if vision.centroidInCenterRect(self.comms.centroidToBump[0], self.comms.centroidToBump[1]):
#             self.count = self.count + 1
#             rospy.loginfo("Count: {}".format(self.count))
        
        # Correct for depth
#         if self.depthCount < 10 and abs(self.comms.deltaY) > 0.010:
#        if not self.depthCorrected and abs(self.comms.deltaY) > 0.010:
#             self.comms.defaultDepth = self.comms.defaultDepth + self.comms.deltaY*self.deltaYMult
#             # Make sure it doesnt surface
#             if self.comms.defaultDepth < 0.1:
#                 self.comms.defaultDepth = 0.1
#             self.comms.sendMovement(depth=self.comms.defaultDepth, blocking=True)
#             self.depthCount = self.depthCount + 1
#             rospy.loginfo("Depth corrected {}".format(self.depthCount))
#             self.depthCorrected = True 
            # pixel radius of buoy seen / screen width * 2 * real radius of bouy * dy / screen width

        if abs(self.comms.deltaY) > 0.010:
            if self.comms.rectArea > 9000:
                self.deltaYMult = 0.05
            self.comms.defaultDepth = self.comms.defaultDepth + self.comms.deltaY*self.deltaYMult
            if self.comms.defaultDepth < 0.1:
                self.comms.defaultDepth = 2.0

        self.comms.sendMovement(sidemove=self.comms.deltaX*self.deltaXMult, 
                                depth=self.comms.defaultDepth, 
                                timeout=0.4, blocking=False)

        return 'centering'

# For bump
class bangBuoy(smach.State):
    deltaXMult = 5.0
    deltaYMult = 0.2
    area = 6500
    count = 0

    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['banging', 'bang_to_center', 'aborted', 'killed'])
        self.comms = comms
        self.curHits = 0
    
    def execute(self, userdata):
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted'
        
        if self.count > 20:           
            return 'bang_to_center'
        
        if self.comms.rectArea > self.area: 
            self.count = self.count + 1
  
        if abs(self.comms.deltaY) > 0.10:
            if self.comms.rectArea > 1500:
                self.deltaYMult = 0.05
            self.comms.defaultDepth = self.comms.defaultDepth + self.comms.deltaY*self.deltaYMult
            # Make sure it doesnt surface
            if self.comms.defaultDepth < 0.1:
                self.comms.defaultDepth = 2.0
  
        # Move forward & correct heading 
        self.comms.sendMovement(forward=0.3, sidemove=self.comms.deltaX*self.deltaXMult,
                                depth=self.comms.defaultDepth,
                                blocking=False)
        return 'banging'

def main():
    rospy.init_node('rgb_buoy_node', anonymous=False)
    rosRate = rospy.Rate(20)
    myCom = Comms()

    rospy.loginfo("RGB Loaded")
    
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'killed'])      
    
    with sm:
        smach.StateMachine.add("DISENGAGE", Disengage(myCom),
                                transitions={'start_complete': "SEARCH",
                                         'killed': 'killed'})
        
        smach.StateMachine.add("SEARCH", Search(myCom),
                               transitions={'search_complete': "BANGBUOY",
                                            'aborted': 'aborted', 
                                            'killed': 'killed'})
    
        smach.StateMachine.add("CENTERING", Centering(myCom),
                               transitions={'centering': "CENTERING",
                                            'centering_complete': 'succeeded',
                                            'aborted': 'aborted',
                                            'killed': 'killed'})
        
        smach.StateMachine.add("BANGBUOY", bangBuoy(myCom),
                               transitions={'banging': "BANGBUOY",
                                            'bang_to_center': "CENTERING",
                                            'aborted': 'aborted',
                                            'killed': 'killed'})
    
    #set up introspection Server
    introServer = smach_ros.IntrospectionServer('mission_server', sm, '/MISSION/RGB_BUOY')
    introServer.start()
    
    sm.execute()
