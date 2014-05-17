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

from vision import RoundVision

# Globals 
locomotionGoal = None 

class Disengage(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['start_complete', 'killed'])
        self.comms = comms
    
    def execute(self, userdata):
        while self.comms.isAborted:
            if self.comms.isKilled:
                return 'killed'
            rospy.sleep(rospy.Duration(0.3))
        
        if isTesting:
            self.comms.register()
            rospy.loginfo("Starting Round")
        
        return 'start_complete'
    
class Search(smach.State):
    timeout = 10
    
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['search_complete', 'aborted', 'killed'])
        self.comms = comms
    
    def execute(self, ud):
        start = time.time()
        while not self.comms.foundSomething:
            if self.comms.isKilled:
                return 'killed'
            if self.comms.isAborted or (time.time() - start) > self.timeout:
                self.comms.isAborted = True
                return 'aborted' 
            
            # Search in figure of 8? 
            rospy.sleep(rospy.Duration(0.3))
        
        return 'search_complete'
    
# Precise movements when near centroid
class Centering (smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['centering', 'centering_complete', 'aborted' 'killed'])
        self.comms = comms
    
    def execute(self, userdata):
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted'
        
# Shoot forward across the rod
class ShootTorpedo(smach.State):
    def __init__(self, comms):
        smach.State.__init__(self, outcomes=['shoot_again', 'shoot_forward_complete', 'aborted' 'killed'])
        self.comms = comms
    
    def execute(self, userdata):
        if self.comms.isKilled:
            return 'killed'
        if self.comms.isAborted:
            return 'aborted'
        
        # Shoot once more 
        if self.comms.numShoot is not 2:
            return 'shoot_again'
        
        return 'shoot_forward_complete'
    
    
def main():
    rospy.init_node('torpedo_node', anonymous=False)
    rosRate = rospy.Rate(20)
    myCom = Comms()

    rospy.loginfo("Torpedo Loaded")
    
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'killed'])      
    
    with sm:
        smach.StateMachine.add("DISENGAGE", Disengage(myCom),
                                transitions={'start_complete': "SEARCH",
                                         'killed': 'killed'})
    
    #set up introspection Server
    introServer = smach_ros.IntrospectionServer('mission_server', sm, '/MISSION/TORPEDO')
    introServer.start()
    
    sm.execute()
    rospy.loginfo(outcomes)
        