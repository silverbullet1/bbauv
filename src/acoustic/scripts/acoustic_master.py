#!/usr/bin/env python
import rospy
import roslib;roslib.load_manifest('acoustic')
import socket
import smach
import signal
from acoustic_brain import AcousticNode
from bbauv_msgs.srv import *

#Global variable
param = rospy.get_param('~test', True)
param2 = rospy.get_param('~tcp', True)

class Disengage(smach.State):
    def __init__(self, ac):
        smach.State.__init__(self, outcomes=['completed', 'init'])
        self.ac = ac
    def execute(self, userdata):
            if self.ac.isTest:
                self.ac.isDormant = False
            while self.ac.isDormant:
                pass
            if self.ac.inTheBox:
                rospy.loginfo("Task completed")
                self.ac.toMission(task_complete_request=True)
                return 'completed'
            elif self.ac.isKilled:
                rospy.loginfo("Task aborted")
                self.ac.toMission(task_complete_request=False)
                return 'aborted'
            else:
                #self.ac.sock.listen(1)
                #(self.ac.TCP_connect, addr) = self.ac.sock.accept()
                self.ac.initAll()
                rospy.loginfo("All services and subscribers started")
                self.ac.sendMovement(depth=1.0)
                rospy.loginfo("Dived")
                return 'init'

class StopListen(smach.State):
    def __init__(self, ac):
        smach.State.__init__(self, outcomes=['calculated'])
        self.ac = ac
    def execute(self, userdata):
        if self.ac.isTest:
            rospy.loginfo("In Test")
            self.ac.elevationAngle = 50
            self.ac.DOA = 110
        else:
            rospy.loginfo("Listening to Pings")
            self.ac.calculateDOA(self.ac.TCP_connect, 1) 
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
        elif self.ac.elevationAngle < 45:
            rospy.loginfo("Found First Pinger, elevation less")
            return 'foundFirst'
        else:
            self.ac.sendMovement(turn=self.ac.DOA)
            self.ac.sendMovement(forward=self.ac.pingerDistance)
            rospy.loginfo("Actions completed")
            return 'holdPosition'

class FindAlt(smach.State):
    def __init__(self, ac):
        smach.State.__init__(self, outcomes=['done'])
        self.ac = ac
    def execute(self, userdata):
        rospy.loginfo("Looking for second pinger")
        self.ac.calculateDOA(TCP_connect, 2)
        self.ac.sendMovement(turn=self.ac.DOA)
        self.ac.sendMovement(forward=1.0)
        self.ac.sendMovement(depth=0.0)
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
                                transitions={'done':'DISENGAGE'})

    outcomes = sm.execute()
    rospy.loginfo(outcomes)
if __name__ == "__main__":
    rospy.init_node("acoustic_master")
    acoustic = AcousticNode(param, param2)
    #Initialise connection to mission planner
    comServer = rospy.Service("/acoustic/mission_to_vision", mission_to_vision, acoustic.handleSrv)
    main()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logerr("Keyboard Interrupt")
