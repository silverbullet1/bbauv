#!/usr/bin/env python
import roslib; roslib.load_manifest('vision')
import rospy

import smach

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from bucket_vision import BucketDetector

#Starts off in disengage class
class Disengage(smach.State):
    timeout = 1500
    
    def __init__(self, bucketDetector):
        smach.State.__init__(self, outcomes=['start_complete', 'task_complete', 'aborted'])
        self.bucketDetector = bucketDetector
    
    def execute(self, userdata):
        self.bucketDetector.unregister()

        #Check for abort or kill signal
        timecount = 0
        while bucketDetector.isAborted:
            if timecount > self.timeout or bucketDetector.isKilled:
                return 'aborted' 
            rospy.sleep(rospy.Duration(0.1))
            timecount += 1

        self.bucketDetector.register()
        return 'start_complete'

#Searches for the bucket
class Searching(smach.State):
    def __init__(self, bucketDetector):
        smach.State.__init__(self, outcomes=['search_complete', 'aborted'])
        self.bucketDetector = bucketDetector
    
    def execute(self, userdata):
        #Check for abort signal
        if self.bucketDetector.isAborted:
            return 'aborted'

        while not self.bucketDetector.rectData['detected']: 
            rospy.sleep(rospy.Duration(0.1))

        #TODO Notify mission planner that bucket is found
        return 'search_complete'

#Centers the robot to the bucket
class Centering(smach.State):
    def __init__(self, bucketDetector):
        smach.State.__init__(self, outcomes=['centering_complete', 'centering',
                                             'lost_bucket', 'aborted'])
        self.bucketDetector = bucketDetector
    
    def execute(self, userdata):
        if self.bucketDetector.isAborted:
            return 'aborted'

        rectData = self.bucketDetector.rectData
        if not rectData['detected']:
            return 'lost_bucket'

        screenWidth = self.bucketDetector.screen['width']
        screenCenterX = screenWidth / 2
        screenHeight = self.bucketDetector.screen['height']
        screenCenterY = screenHeight / 2
        deltaX = (rectData['centroid'][0] - screenCenterX) / screenWidth
        deltaY = (rectData['centroid'][1] - screenCenterY) / screenHeight
        rospy.loginfo("x-off: %lf, y-off: %lf", deltaX, deltaY)
        
        if abs(deltaX) < 0.05 and abs(deltaY) < 0.05:
            return 'centering_complete'

        fwd_setpoint = -deltaY * 50.0
        sm_setpoint = deltaX * 50.0
        self.bucketDetector.sendMovement(f=fwd_setpoint, sm=sm_setpoint)
        return 'centering'

#Fire the gold ball
class Firing(smach.State):
    def __init__(self, bucketDetector):
        smach.State.__init__(self, outcomes=['firing_complete', 'aborted'])
        self.bucketDetector = bucketDetector
    
    def execute(self, userdata):
        self.bucketDetector.stopRobot()
        #TODO fire the ball 
        #TODO notify mission planner that bucket task is done
        return 'aborted'

if __name__ == '__main__':
    rospy.init_node('bucketdetector')
    isTestMode = rospy.get_param('~testing', False)
    bucketDetector = BucketDetector()
    rospy.loginfo("Bucket loaded!")
    
    if isTestMode:
        bucketDetector.isAborted = False
        
    sm = smach.StateMachine(outcomes=['task_complete', 'aborted'])
    with sm:
        smach.StateMachine.add('DISENGAGE', Disengage(bucketDetector),
                               transitions={'start_complete' : 'SEARCHING',
                                            'aborted':'aborted',
                                            'task_complete':'task_complete'})
        smach.StateMachine.add('SEARCHING', Searching(bucketDetector),
                               transitions={'search_complete' : 'CENTERING',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('CENTERING', Centering(bucketDetector),
                               transitions={'centering_complete':'FIRING',
                                            'centering':'CENTERING',
                                            'lost_bucket':'SEARCHING',
                                            'aborted':'DISENGAGE'})
        smach.StateMachine.add('FIRING', Firing(bucketDetector),
                               transitions={'firing_complete' : 'task_complete',
                                            'aborted':'DISENGAGE'})
    
    outcomes = sm.execute()
    rospy.loginfo(outcomes)
