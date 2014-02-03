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
    timeout = 1500
    
    def __init__(self, bucketDetector):
        smach.State.__init__(self, outcomes=['start_complete', 'task_complete', 'aborted'])
    
    def execute(self, userdata):
        bucketDetector.unregister()
        #Check for abort or kill signal
        while not bucketDetector.isAborted:
            if timecout > timeout or bucketDetector.isKilled:
                return 'aborted' 
        return 'start'

#Searches for the bucket
class Searching(smach.State):
    def __init__(self, bucketDetector):
        smach.State.__init__(self, outcomes['search_complete', 'aborted'])
    
    def execute(self, userdata):
        return 'search_complete'

#Centers the robot to the bucket
class Centering(smach.State):
    def __init__(self, bucketDetector):
        smach.State.__init__(self, outcomes['centering_complete', 'aborted'])
    
    def execute(self, userdata):
        return 'centering_complete'

#Fire the gold ball
class Firing(smach.State):
    def __init__(self, bucketDetector):
        smach.State.__init__(self, outcomes['firing_complete', 'aborted'])
    
    def execute(self, userdata):
        return 'firing_complete'

if __name__ == '__main__':
    rospy.init_node('bucketdetector')
    isTest = rospy.get_param('~testing', False)
    bucketDetector = BucketDetector()
    rospy.loginfo("Bucket loaded!")
    
    sm = smach.StateMachine(outcomes=['task_complete', 'aborted'])
    with sm:
        smach.StateMachine.add('DISENGAGE', Disengage(bucketDetector),
                               transitions={'start_complete' : 'SEARCHING',
                                            'aborted':'aborted',
                                            'task_complete':'task_complete'})
        smach.StateMachine.add('SEARCHING', Seaching(bucketDetector),
                               transitions={'search_complete' : 'CENTERING',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('CENTERING', Centering(bucketDetector),
                               transitions={'centering_complete':'FIRING',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('FIRING', Firing(bucketDetector),
                               transitions={'firing_complete' : 'DISENGAGE',
                                            'aborted':'DISENGAGE'})
    
    if isTestMode:
        bucketDector.isAborted = False
        
    outcomes = sm.execute()
    rospy.loginfo(outcomes)