#!/usr/bin/env python
import rospy
import roslib
import socket
import smach
import smach_ros
import time
from comm import Comm
from acousticStream import AcousticStream
import threading

#GLOBAL

class Disengage(smach.State):
    def __init__(self, ac):
        smach.State.__init__(self, outcomes=['completed', 'init', 'aborted'])
        self.ac = ac

    def execute(self, userdata):
            if self.ac.isDone:
                rospy.loginfo("Task completed")
                self.ac.streamer.socket.close()
                if not self.ac.isAlone:
                    self.ac.toMission(task_complete_request=True, fail_request=False)
                return 'completed'
            elif self.ac.isAborted:
                rospy.loginfo("Task aborted")
                self.ac.streamer.socket.close()
                if not self.ac.isAlone:
                    self.ac.toMission(task_complete_request=False, fail_request=True)
                return 'aborted'
            else:
                while not self.ac.isStart: 
                    rospy.sleep(rospy.Duration(0.5))
                self.ac.kickstart()
                self.ac.listen()
                rospy.loginfo("init successful")
                rospy.loginfo("All services and subscribers started")
                #Wait for vehicle to stabilise
                time.sleep(5)
                return 'init'

class StopListen(smach.State):
    def __init__(self, ac):
        smach.State.__init__(self, outcomes=['calculated', 'aborted'])
        self.ac = ac

    def execute(self, userdata):
    	if(self.ac.isAborted):
            return 'aborted'
        else:
			self.ac.adjustStep()
			rospy.loginfo("Listening to Pings")
			#Wait to get most updated DOA
			time.sleep(self.ac.initDuration)
			return 'calculated'

class Moving(smach.State):
    def __init__(self, ac):
        smach.State.__init__(self, outcomes=['aborted', 'holdPosition', 'done']) 
        self.ac = ac
    def execute(self, userdata):
    	if(self.ac.isAborted):
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
            if self.ac.isAlone:
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
    rospy.init_node("AcousticMaster")
    rospy.loginfo("Initialised")
    sm = smach.StateMachine(outcomes=['failed', 'surface']) 
    sm_server = smach_ros.IntrospectionServer("/acoustic/master", sm, "/PING")
    sm_server.start()
    acoustic = Comm()
    rospy.loginfo("Done with comm")

    with sm:
        smach.StateMachine.add("DISENGAGE", Disengage(acoustic),
                               transitions={'init':'STOP_LISTEN','completed':'surface', 'aborted':'failed'})
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
