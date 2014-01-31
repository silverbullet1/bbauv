#!/usr/bin/env python

import roslib; roslib.load_manifest('vision')
import rospy

import sys
import smach
import smach_ros
import math
import os

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *

import numpy as np
from rospy.timer import sleep

from linefollower_vision import LineFollower

class Disengage(smach.State):
    def __init__(self, lf):
        smach.State.__init__(self, outcomes=['start_complete', 'complete_outcome', 'aborted'])
        self.linefollower = lf

    def execute(self, userdata):
        return 'start_complete'

class Searching(smach.State):
    def __init__(self, lf):
        smach.State.__init__(self, outcomes=['line_found', 'aborted'])
        self.linefollower = lf

    def execute(self, userdata):
        return 'line_found'
    
class FollowingLine(smach.State):
    def __init__(self, lf):
        smach.State.__init__(self, outcomes=['lost_line', 'aborted'])
        self.linefollower = lf

    def execute(self, userdata):
        return 'lost_line'

def main():
    rospy.init_node("linefollower")
    linefollower = LineFollower();
    

    #Creating a State Machine Container
    sm = smach.StateMachine(outcomes=['complete_task', 'aborted']) 

    with sm:
        smach.StateMachine.add("DISENGAGE", Disengage(linefollower),
                               transitions={'start_complete':'SEARCHING',
                                            'complete_outcome':'complete_task',
                                            'aborted':'aborted'})

        smach.StateMachine.add("SEARCHING", Searching(linefollower),
                               transitions={'line_found':'FOLLOWINGLINE'})

        smach.StateMachine.add("FOLLOWINGLINE", FollowingLine(linefollower),
                               transitions={'lost_line':'SEARCHING',
                                            'aborted':'aborted'})
    outcomes = sm.execute()

    rospy.spin()

if __name__ == "__main__":
    main()
