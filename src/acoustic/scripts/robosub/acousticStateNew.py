#!/usr/bin/env python
import rospy
import roslib
import socket
import smach
import smach_ros
import signal
import time
from comm import Comm
from acoustic_stream import AcousticStream

#GLOBAL

test = rospy.get_param("~test", False)

class Disengage(smach.State):
    def __init__(self, ac):
        smach.State.__init__(self, outcomes=['completed', 'init'])
        self.ac = ac

    def execute(self, userdata):
            if self.ac.isDone:
                rospy.loginfo("Task completed")
                if not self.ac.isAlone:
                    self.ac.toMission(task_complete_request=True, fail_request=False)
                return 'completed'
            elif self.ac.isAborted:
                rospy.loginfo("Task aborted")
                if not self.ac.isAlone:
                    self.ac.toMission(task_complete_request=False, fail_request=True)
                return 'aborted'
            else:
                while not self.ac.isStart:
                    rospy.sleep(rospy.Duration(0.5))
                self.ac.kickstart()
                rospy.loginfo("init successful")
                rospy.loginfo("All services and subscribers started")
                #Wait for vehicle to stabilise
                time.sleep(5)
                return 'init'

class Moving(smach.State):
    def __init__(self, ac):
        smach.State.__init__(self, outcomes=['aborted',  'done']) 
        self.ac = ac
    def execute(self, userdata):
        self.ac.sendMovement(forward=0.0, turn=self.ac.doa)
    	while self.ac.elevation > 30:
            if(self.ac.isAborted):
                rospy.loginfo("Task aborted")
                return 'aborted'
            else:
                rospy.loginfo("Turned")
                self.ac.sendMovement(forward=0.0, turn=self.ac.doa, wait=False)
                self.ac.sendMovement(forward=4.0, wait=False)
                rospy.loginfo("Forward")
                rospy.loginfo("Actions completed")
        return 'done'

        

def main():
    #Creating a State Machine Container
    rospy.init_node("acoustic_master")
    rospy.loginfo("Initialised")
    sm = smach.StateMachine(outcomes=['foundPinger', 'surface']) 
    sm_server = smach_ros.IntrospectionServer("/acoustic/master", sm, "/PING")
    sm_server.start()
    acoustic = Comm()
    signal.signal(signal.SIGINT, acoustic.quit)
    rospy.loginfo("Done with comm")

    with sm:
        smach.StateMachine.add("DISENGAGE", Disengage(acoustic),
                               transitions={'init':'MOVING','completed':'surface' })
        smach.StateMachine.add("MOVING", Moving(acoustic),
                                transitions={'aborted':'DISENGAGE','done':'DISENGAGE'})

    outcomes = sm.execute()
    rospy.loginfo(outcomes)

if __name__ == "__main__":
    main()
    #Initialise connection to mission planner
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logerr("Keyboard Interrupt")
