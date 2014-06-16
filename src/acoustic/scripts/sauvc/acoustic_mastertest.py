#!/usr/bin/env python
import rospy
import roslib;roslib.load_manifest('acoustic')
import socket
import smach
import signal
from acoustic_braintest import AcousticNode
from bbauv_msgs.srv import *
from bbauv_msgs.msg import *

#Global variable

class Disengage(smach.State):
    def __init__(self, ac):
        smach.State.__init__(self, outcomes=['completed', 'init'])
        self.ac = ac

    def execute(self, userdata):
            if self.ac.isTest:
                self.ac.isDormant = False
            while self.ac.isDormant:
                rospy.sleep(rospy.Duration(1.0))
            if self.ac.inTheBox:
                rospy.loginfo("Task completed")
                self.ac.toMission(task_complete_request=True)
                return 'completed'
            elif self.ac.isKilled:
                rospy.loginfo("Task aborted")
                self.ac.toMission(task_complete_request=False)
                return 'aborted'
            else:
                self.ac.initAll()
                rospy.loginfo("init successful")
                #self.ac.sendMovement(forward=0.0,turn=60, depth=1.0, absolute=True)
                self.ac.s.listen(1)
                (self.ac.TCP_connect, addr) = self.ac.s.accept()
                rospy.loginfo("All services and subscribers started")
                return 'init'

class StopListen(smach.State):
    def __init__(self, ac):
        smach.State.__init__(self, outcomes=['calculated'])
        self.ac = ac
    def execute(self, userdata):
        rospy.loginfo("Listening to Pings")
        while self.ac.isTest:
            self.ac.calculateDOA(self.ac.TCP_connect,1) 
        return 'calculated'

class Moving(smach.State):
    def __init__(self, ac):
        smach.State.__init__(self, outcomes=['holdPosition', 'foundFirst']) 
        self.ac = ac
    def execute(self, userdata):
        if self.ac.counter > 1 and self.ac.overShotPinger(self.ac.DOA):
            rospy.loginfo("Found First Pinger, overshot")
            self.ac.sendMovement(forward=self.ac.pingerDistance)
            rospy.loginfo("Reversed: " + str(self.ac.pingerDistance))
            return 'foundFirst'
        elif self.ac.elevationAngle < 30: 
            rospy.loginfo("Found First Pinger, elevation less")
            return 'foundFirst'
        else:
            rospy.loginfo("Doing movement\n")
            theta = self.ac.DOA
            self.ac.sendMovement(forward=0.0, turn=theta, depth=1.0)
            rospy.loginfo("Turned")
            self.ac.sendMovement(forward=self.ac.pingerDistance)
            rospy.loginfo("Forward")
            rospy.loginfo("Actions completed")
            return 'holdPosition'

class FindAlt(smach.State):
    def __init__(self, ac):
        smach.State.__init__(self, outcomes=['done', 'fail'])
        self.ac = ac
    def execute(self, userdata):
        if self.ac.isKilled:
            self.ac.sendMovement(depth=-0.1)
            return 'fail'
        else:
            rospy.loginfo("Looking for second pinger")
            self.ac.sendMovement(forward=0.0, sidemove=0.2, depth=1.0)
            self.ac.sendMovement(depth=-0.1)
            self.ac.inTheBox = True
            return 'done'
        

#utility Functions


def main():
    #Creating a State Machine Container
    sm = smach.StateMachine(outcomes=['foundPinger', 'surface']) 

    with sm:
        smach.StateMachine.add("DISENGAGE", Disengage(acoustic),
                               transitions={'init':'STOP_LISTEN','completed':'surface' })
        smach.StateMachine.add("STOP_LISTEN", StopListen(acoustic),
                                transitions={'calculated':'MOVING'})
        smach.StateMachine.add("MOVING", Moving(acoustic),
                                transitions={'foundFirst':'FINDALT','holdPosition':'STOP_LISTEN'})
        smach.StateMachine.add("FINDALT", FindAlt(acoustic),
                                transitions={'done':'DISENGAGE', 'fail':'DISENGAGE'})

    outcomes = sm.execute()
    rospy.loginfo(outcomes)

if __name__ == "__main__":
    rospy.init_node("acoustic_master")
    param = rospy.get_param('~test', False)
    rospy.loginfo("Initialised")
    acoustic = AcousticNode(param)
    comServer = rospy.Service("/acoustic/mission_to_vision", mission_to_vision, acoustic.handleSrv)
    main()
    #Initialise connection to mission planner
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown("Died")
        rospy.logerr("Keyboard Interrupt")
