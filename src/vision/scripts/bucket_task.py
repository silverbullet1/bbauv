'''
Bucket task state machine 
'''

import roslib
import rospy
import actionlib
from rospy.timer import sleep

import smach
import smach_ros

from dynamic_reconfigure.server import Server
#from Vision.cfg import bucketTaskConfig

import math
import os
import sys
import numpy as np

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
import bucketvision

#Starts off in disengage class
class Disengage(smach.State):
    client = None
    def __init__(self):
        smach.State.__init__(self, outcomes=['start', 'complete', 'aborted'], input_keys=['complete'])
    
    def execute(self, userdata):
        #do stuff
        return 'complete'

#Searches for the bucket
class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes['search_complete', 'aborted', 'mission_abort'])
    
    def execute(self, userdata):
        #do stuff
        return 'search_complete'

#Centers the robot to the bucket
class Centering(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes['centering_complete', 'aborted', 'mission_abort'], output_keys=['center_pos'])
    
    def execute(self, userdata):
        #do stuff
        return 'centering_complete'

#Manuoevre the robot towards the bucket center  
class Manuoevre(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes['manuoevre_complete', 'aborted', 'mission_abort'])
    def execute(self,userdata):
        #do stuff
        return 'manuoevre_complete'

#Aims the golf ball
class Aiming(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes['aiming_complete', 'aborted', 'mission_abort'], output_keys=['center_pos'])
    
    def execute(self, userdata):
        #do stuff
        return 'aiming_complete'

#Fire the gold ball
class Firing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes['firing_complete', 'aborted', 'mission_abort'])
    
    def execute(self, userdata):
        #do stuff
        return 'firing_complete'
'''
Main python thread
'''

def handle_srv(req):
    global isStart
    global isAbort
    global locomotionGoal
    global bkt
    
    rospy.loginfo("Bucket service handled")
    
    if req.start_request:
        rospy.loginfo("Bucket is Start")
        isStart = True
        isAbort = False 
        #locomotionGoal = req.start_ctrl
    if req.abort_reqest:
        rospy.loginfo("Bucket abort received")
        isAbort = True
        isStart = False
        bkt.unregister()
    
    #To fill accordingly
    return mission_to_visionResponse(isStart, isAbort)
    
#Global variables 
isStart = False 
isAbort = False
isEnd = False 
rosRate = None 
isTest = False
bkt = None
VisionLoopCount = 0                 #Counter for number of times the image is being processed

bucket_params = {'bucket_area':0, 'firing_x':10, 'firing_y':10, 'centering_x':0, 'centering_y':0, 'aiming_x':0, 'aiming_y':0}
mani_pub = None
movement_client = None
locomotionGoal = None

#Then set up the param configuration window
def bucketCallback(config, level):
    return config

if __name__ == '__main__':
    rospy.init_node('Bucket', anonymous=False)
    isTest = rospy.get_param('~testmode', False)
    rosRate = rospy.Rate(20)
    bkt = Bucket(False)
    rospy.loginfo("Bucket loaded!")
    
    if isTestMode:
        isStart = True
    
    #Link to motion
    movement_client = actionlib.SimpleActionClient('LocomotionServer', bbauv_msgs.msg.ControllerAction)
    movement_client.wait_for_server()
    mani_pub = rospy.Publisher("/manipulators", manipulator)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down flare")
    pass
    




