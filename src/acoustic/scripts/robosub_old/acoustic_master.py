#!/usr/bin/env python
import rospy
import roslib
import socket
import smach
import signal
import time
from comm import Comm
import threading
from acoustic_stream import AcousticStream

class Disengage(smach.State):
    def __init__(self, ac):
        smach.State.__init__(self, outcomes=['completed', 'init'])
        self.ac = ac
        self.stream = AcousticStream()

    def execute(self, userdata):
            if self.ac.isDone:
                rospy.loginfo("Task completed")
                if self.ac.isAlone:
                    self.ac.toMission(task_complete_request=True)
                rospy.signal_shutdown("Completed")
                return 'completed'
            elif self.ac.isKilled:
                rospy.loginfo("Task aborted")
                if self.ac.isAlone:
                    self.ac.toMission(task_complete_request=False)
                return 'aborted'
            else:
                self.ac.kickstart()
                rospy.loginfo("init successful")
            	#Initial Movement
                self.ac.sendMovement(forward=0.0, depth=1.0)
                rospy.loginfo("All services and subscribers started")
                return 'init'

class StopListen(smach.State):
    def __init__(self, ac):
        smach.State.__init__(self, outcomes=['calculated', 'aborted'])
        self.ac = ac

    def execute(self, userdata):
    	if(self.ac.isKilled):
            return 'aborted'
        else:
			rospy.loginfo("Listening to Pings")
			#Wait to get most updated DOA
			time.sleep(self.ac.initDuration)
			return 'calculated'

class Moving(smach.State):
    def __init__(self, ac):
        smach.State.__init__(self, outcomes=['aborted', 'holdPosition', 'done']) 
        self.ac = ac
    def execute(self, userdata):
    	if(self.ac.isKilled):
			rospy.loginfo("Task aborted")
			return 'aborted'
        elif self.ac.count > 1 and self.ac.overShotPinger(self.ac.doa):
            rospy.loginfo("Found Pinger, overshot")
            #self.ac.sendMovement(forward=0.0, sidemove=-0.2, depth=1.0)
            self.ac.sendMovement(forward=-(self.ac.step_size/2))
            self.ac.isDone = True
            return 'done'
        elif self.ac.elevation < self.ac.e_low and self.ac.count > 1: 
            rospy.loginfo("Found First Pinger, elevation less")
            self.ac.isDone = True
            return 'done'
        else:
            self.ac.initDuration = 1
            self.ac.count += 1
            rospy.loginfo("Doing movement")
            self.ac.sendMovement(forward=0.0, turn=self.ac.doa)
            rospy.loginfo("Turned")
            self.ac.sendMovement(forward=self.ac.step_size)
            rospy.loginfo("Forward")
            rospy.loginfo("Actions completed")
            return 'holdPosition'

        

def main():
    #Creating a State Machine Container
    rospy.init_node("acoustic_master")
    rospy.loginfo("Initialised")
    sm = smach.StateMachine(outcomes=['foundPinger', 'surface']) 
    acoustic = Comm()
    rospy.loginfo("Done with comm")

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
    main()
    #Initialise connection to mission planner
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logerr("Keyboard Interrupt")
