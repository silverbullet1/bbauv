'''
Bucket task state machine 
'''

import roslib; roslib.load_manifest("vision")
import rospy

import smach

import math

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from bucket_vision import BucketDetector

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

#Fire the gold ball
class Firing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes['firing_complete', 'aborted', 'mission_abort'])
    
    def execute(self, userdata):
        #do stuff
        return 'firing_complete'

if __name__ == '__main__':
    rospy.init_node('Bucket', anonymous=False)
    isTest = rospy.get_param('~testmode', False)
    rosRate = rospy.Rate(20)
    bucketDetector = BucketDetector(False)
    rospy.loginfo("Bucket loaded!")
    
    if isTestMode:
        bucketDector.isAborted = False
