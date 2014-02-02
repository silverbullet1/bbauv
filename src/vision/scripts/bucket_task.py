'''
Bucket task state machine 
'''

import roslib; roslib.load_manifest("vision")
import rospy
import actionlib

import smach

import math

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
import bucket_vision

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
    
if __name__ == '__main__':
    rospy.init_node('Bucket', anonymous=False)
    isTest = rospy.get_param('~testmode', False)
    rosRate = rospy.Rate(20)
    bucketDetector = BucketDetector(False)
    rospy.loginfo("Bucket loaded!")
    
    if isTestMode:
        bucketDector.isAborted = False
            
    #Link to motion
    movement_client = actionlib.SimpleActionClient('LocomotionServer', bbauv_msgs.msg.ControllerAction)
    movement_client.wait_for_server()
    mani_pub = rospy.Publisher("/manipulators", manipulator)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down flare")
    pass
