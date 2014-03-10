#!/usr/bin/env python
#Reverse movement when validating result
#Testing by going to pinger and calculate angle to pinger and list side by side with labview data
import rospy
import roslib;roslib.load_manifest('acoustic')
import socket
import smach
import signal
from acoustic_node import AcousticNode


isKilled = False

class Disengage(smach.State):
    def __init__(self, ac):
        self.ac = ac
        smach.State.__init__(self, outcomes=['initialised', 'aborted', 'test', 'eureka', 'active'])
    def execute(self, userdata):
        rospy.loginfo("In Disengage")
        if not self.ac.aborted and not self.ac.killed:
            self.ac.startSrv_Sub()
            if self.ac.test:
                return 'test'
            elif self.ac.active:
                self.ac.aborted = False
                return 'active'
            else:
                rospy.loginfo("All services and subscribers setup")
                return 'initialised'
        elif self.ac.completed:
            return 'completed'
        else:
            self.ac.abortMission()
            return 'aborted'
            
class ActiveReading(smach.State):
    def __init__(self, ac):
        smach.State.__init__(self, outcomes=['moving','identified_pinger', 'failed'])
        self.ac = ac
    def execute(self, userdata):
        rospy.loginfo("Executing main task")
        self.ac.getFinalPoints(conn)
        if len(self.ac.finalDestination) is not 2:
            return 'failed'
        else:
            self.ac.taskComplete()
            return 'identified_pinger'

#Class functions 
def quitProgram():
    global isKilled
    isKilled = True

def main():
    global ros_param
    #Check if killed
    signal.signal(signal.SIGINT, quitProgram)
    
    #Publish labview data
    while not acoustic.aborted:
        acoustic.angle_pub.publish(acoustic.DOA) 

    #Creating a State Machine Container
    sm = smach.StateMachine(outcomes=['found_pinger', 'aborted']) 

    with sm:
        smach.StateMachine.add("DISENGAGE", Disengage(acoustic),
                               transitions={'initialised':'ACTIVE__READING', 'aborted':'aborted', 'eureka':'found_pinger'})


        smach.StateMachine.add("ACTIVE_READING", ActiveReading(acoustic),
                                transitions={'moving':'ACTIVE_READING',
                                             'failed':'DISENGAGE', 
                                             'identified_pinger':'found_pinger'})

    outcomes = sm.execute()
    rospy.loginfo(outcomes)

if __name__ == "__main__":
    rospy.init_node("acoustic_task")
    ros_param = {'test':rospy.get_param('~test', False), 
                'boundary':rospy.get_param('~boundary', 30),
                'sidemove':rospy.get_param('~sidemove', 1),
                'takes':rospy.get_param('~takes', 5),
                'active':rospy.get_param('~active', False)}
    acoustic = AcousticNode(ros_param)
    main()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logerr("Keyboard Interrupt")
