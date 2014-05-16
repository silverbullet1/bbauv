'''
Smach state machine for torpedo
'''

import roslib; roslib.load_manifest('vision')
import rospy

import smach, smach_ros

from comms import Comms

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from dynamic_reconfigure.server import Server

from vision import TorpedoVision

# Globals 
locomotionGoal = None 

class Disengage(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['start_complete', 'aborted', 'killed'])
        self.comms = comms
        
    def execute(self, userdata):
        while not self.comms.foundRedPeg:
            if self.comms.isKilled:
                rospy.signal_shutdown("Bye")
                return 'killed'
            if self.comms.isAborted:
                rospy.signal_shutdown("User aborted")
                return 'aborted'
            
            rospy.sleep(rospy.Duration(0.3))
    
        self.comms.register()
        if self.comms.isAlone:
            self.comms.inputHeading = self.comms.curHeading
                
        return 'start_complete'

class Search(smach.State):
    timeout = 120
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['search_complete', 'timeout', 'aborted'])
        self.comms = comms
        
    def execute(self, userdata):
        start = time.time()
        
        while not self.comms.foundCircles:
            if (time.time() - start) > self.timeout:
                self.comms.isAborted = True
                return 'timeout'
            if self.comms.isKilled or self.comms.isAborted:
                return 'aborted'
        
        return 'search_complete'
    
class Forward(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['forward', 'forward_complete', 'lost', 'aborted'])
        self.comms = comms
        
    def execute(self, userdata):       
        if self.comms.isKilled or self.comms.isAborted:
            return 'aborted'
        
        if len(self.comms.circleData) == 0:
            return 'lost'
        
        if self.comms.circleArea > 10000:
            return 'forward_complete' 
    
        return 'forward'
    
class Centering(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['centering', 'centering_complete', 'lost', 'aborted'])
        self.comms = comms
        
    def execute(self, userdata):       
        if self.comms.isKilled or self.comms.isAborted:
            return 'aborted'
        
        if len(self.comms.circleData) == 0:
            return 'lost'
        
        if self.comms.circleArea > 10000:
            return 'centering_complete'
        
        return 'centering' 
    
class Fire(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['firing_complete', 'task_complete', 'aborted'])
        self.comms = comms
        
    def execute(self, userdata):       
        if self.comms.isKilled or self.comms.isAborted:
            return 'aborted'
        
        # Fire torpedo
        rospy.loginfo("Torpedo fired!")
        self.comms.numShoot = self.comms.numShoot + 1
        
        if self.comms.numShoot == 2:
            return 'task_complete'
        
        return 'firing_complete'   
    
def main():
    rospy.init_node('torpedo_node', anonymous=False)
    rosRate = rospy.Rate(20)
    myCom = Comms()

    rospy.loginfo("Torpedo Loaded")
    
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'killed'])      
    
    with sm:
        smach.StateMachine.add("DISENGAGE", Disengage(myCom),
                                transitions={'start_complete': "SEARCH",
                                             'aborted': 'aborted',
                                             'killed': 'killed'})
        
        smach.StateMachine.add("SEARCH", Search(myCom),
                                transitions={'search_complete': "SEARCH",
                                             'timeout': 'killed',
                                             'killed': 'killed'})   
        
        smach.StateMachine.add("FORWARD", Forward(myCom),
                                transitions={'forward': "FORWARD",
                                             'forward_complete': "CENTERING",
                                             'lost': "SEARCH",
                                             'aborted': 'aborted'})             
    
        smach.StateMachine.add("CENTERING", Centering(myCom),
                                transitions={'centering': "CENTERING",
                                             'centering_complete': "FIRING",
                                             'lost': "SEARCH",
                                             'aborted': 'aborted'})  
        
        smach.StateMachine.add("FIRING", Firing(myCom),
                                transitions={'firing_complete': "SEARCH",
                                             'task_complete': 'succeeded',
                                             'aborted': 'aborted'})  
    
    #set up introspection Server
    introServer = smach_ros.IntrospectionServer('mission_server', sm, '/MISSION/TORPEDO')
    introServer.start()
    
    sm.execute()
    rospy.loginfo(outcomes)
        