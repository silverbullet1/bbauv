#!/usr/bin/env python
import rospy
import roslib;roslib.load_manifest('acoustic')
import socket
import smach
import signal
import time
from comm import Comm

com = Comm()

class Disengage(smach.State):
    def __init__(self, ac):
        smach.State.__init__(self, outcomes=['completed', 'init'])
        self.ac = ac

    def execute(self, userdata):
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
            	#Initial Movement
                self.ac.sendMovement(forward=0.0,turn=60, depth=1.0, absolute=True)
                time.sleep(5)
                rospy.loginfo("All services and subscribers started")
                return 'init'

class StopListen(smach.State):
    def __init__(self, ac):
        smach.State.__init__(self, outcomes=['calculated'])
        self.ac = ac
    def execute(self, userdata):
    	if(self.ac.isKilled):
            return 'aborted'
        else:
			rospy.loginfo("Listening to Pings")
			#Wait to get most updated DOA
			time.sleep(5)
			return 'calculated'

class Moving(smach.State):
    def __init__(self, ac):
        smach.State.__init__(self, outcomes=['holdPosition', 'done']) 
        self.ac = ac
    def execute(self, userdata):
    	if(self.ac.isKilled):
			rospy.loginfo("Task aborted")
			self.ac.toMission(task_complete_request=False)
			return 'aborted'
        elif self.ac.counter > 1 and self.ac.overShotPinger(doa):
            rospy.loginfo("Found Pinger, overshot")
            self.ac.sendMovement(forward=0.0, sidemove=-0.2, depth=1.0)
            self.ac.sendMovement(forward=-0.8)
            self.ac.inTheBox = True
            return 'done'
        elif self.ac.elevationAngle < self.ac.elevationBound and self.ac.counter > 1: 
            rospy.loginfo("Found First Pinger, elevation less")
            return 'done'
        else:
            self.ac.counter += 1
            rospy.loginfo("Doing movement\n")
            self.ac.sendMovement(forward=0.0, turn=doa, depth=1.0)
            rospy.loginfo("Turned")
            self.ac.sendMovement(forward=self.ac.pingerDistance)
            rospy.loginfo("Forward")
            rospy.loginfo("Actions completed")
            return 'holdPosition'

        

#utility Functions

def rawPingHandler(data):
	global doa, elevationAngle, iteration
	doa = data.doa
	elevationAngle = data.elevation 
	iteration = data.iteration


def main():
    #Creating a State Machine Container
    sm = smach.StateMachine(outcomes=['foundPinger', 'surface']) 

    with sm:
        smach.StateMachine.add("DISENGAGE", Disengage(acoustic),
                               transitions={'init':'STOP_LISTEN','completed':'surface' })
        smach.StateMachine.add("STOP_LISTEN", StopListen(acoustic),
                                transitions={'calculated':'MOVING', 'aborted':'DISENGAGE'})
        smach.StateMachine.add("MOVING", Moving(acoustic),
                                transitions={'aborted':'DISENGAGE','holdPosition':'STOP_LISTEN', 'done':'DISENGAGE'})

    outcomes = sm.execute()
    rospy.loginfo(outcomes)

if __name__ == "__main__":
    rospy.init_node("acoustic_master")
    rospy.loginfo("Initialised")
    com = Comm()
    main()
    #Initialise connection to mission planner
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logerr("Keyboard Interrupt")
