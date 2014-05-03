#!/usr/bin/env python

import rospy
import roslib;roslib.load_manifest('acoustic')
import socket
import smach
import signal
from plot_lynnette import Plot

import actionlib
from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from nav_msgs.msg import Odometry
from std_msgs.msg._Float32 import Float32

DOA = 0
Elevation = 1000
numberData = 0

class Disengage(smach.State):
    def __init__(self, acoustic_task):
        self.ac = acoustic_task
        smach.State.__init__(self, outcomes=['start', 'aborted'])
        
    def execute(self, userdata):
        rospy.loginfo("In Disengage")
        
        rospy.sleep(rospy.Duration(0.2))
        return 'start'

class Stop(smach.State):
    
    def __init__(self, acoustic_task):
        self.ac = acoustic_task
        smach.State.__init__(self, outcomes=['waiting_ping', 'received_ping', 'below_ping', 'aborted'])
        
    def execute(self, userdata):
        rospy.loginfo("Stop and wait for ping")
        
        timecount = 0
        timeUp = 1000   #Give some time before a ping reaches
        DOA = 0
        Elevation = 100
        
        data = []
                
        while True:
            self.ac.DOA = self.ac.DOA_music
            self.ac.Elevation = self.ac.Elevation_music
#             self.ac.DOA = self.ac.DOA_classical
#             self.ac.Elevation = self.ac.Elevation_classical
            
            if self.ac.pingDetected:
                rospy.loginfo("Ping detected")
                data.append((self.ac.DOA, self.ac.Elevation))
            
                if len(data) == 2:
                    DOA = data[1][0]
                    Elevation = data[1][1]
                    data = []
                                        
                    rospy.loginfo("DOA")
                    print DOA
                    print Elevation
                
                    if Elevation < 10:
                        return 'below_ping'
                    else: 
                        return 'received_ping'
        
        return 'waiting_ping'

        
class Move(smach.State):
    def __init__(self, acoustic_task):
        self.ac = acoustic_task
        smach.State.__init__(self, outcomes=['complete', 'aborted'])
        
    def execute(self, userdata):
        self.ac.pingDetected = False
        rospy.loginfo("Moving forward, DOA:{}".format(self.ac.DOA))
        self.ac.sendMovement(turn = self.ac.DOA)
        self.ac.sendMovement(forward = 2.0)
        return 'complete'
    
class Stop_And_Surface(smach.State):
    def __init__(self, acoustic_task):
        self.ac = acoustic_task
        smach.State.__init__(self, outcomes=['complete', 'aborted'])
        
    def execute(self, userdata):
        rospy.loginfo("Surfacing")
        self.ac.sendMovement(depth=0.0)
        return 'complete'
        
if __name__ == '__main__':
    rospy.init_node("Acoustics", anonymous=False)
    rosRate = rospy.Rate(20)
    acoustic_task = Plot()
    
    sm = smach.StateMachine(outcomes=['complete', 'aborted'])
    
    with sm:
        smach.StateMachine.add("DISENGAGE", Disengage(acoustic_task),
                               transitions={'start': "STOP",
                                            'aborted': 'aborted' })
        smach.StateMachine.add("STOP", Stop(acoustic_task),
                               transitions={'waiting_ping': "STOP",
                                            'received_ping': "MOVE",
                                            'below_ping': "STOP_AND_SURFACE",
                                            'aborted': 'aborted'})
        smach.StateMachine.add("MOVE", Move(acoustic_task),
                               transitions={'complete': "STOP",
                                            'aborted': 'aborted'})
        smach.StateMachine.add("STOP_AND_SURFACE", Stop_And_Surface(acoustic_task),
                               transitions = {'complete': 'complete',
                                              'aborted': 'aborted'})
    
    outcomes = sm.execute()