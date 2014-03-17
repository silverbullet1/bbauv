#!/usr/bin/env python
#Reverse movement when validating result
#Testing by going to pinger and calculate angle to pinger and list side by side with labview data
import rospy
import roslib;roslib.load_manifest('acoustic')
import socket
import smach
import signal
from acoustic_node import AcousticNode

#Global variable
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
conn = 0
TCP_IP = '192.168.1.149'
TCP_PORT = 5100
BUFFER_SIZE = 60        
LOGFILE = "log.txt"

isKilled = False

class PassiveReading(smach.State):
    global conn
    def __init__(self, ac):
        smach.State.__init__(self, outcomes=['completed', 'failed'])
        self.ac = ac
    def execute(self, userdata):
        if not self.ac.active:
            rospy.loginfo("Taking values passively")
            self.ac.PassiveReading(conn)
        elif self.ac.aborted or self.ac.killed:
            return 'failed' 
        else:
            return 'completed'

class UsePredicted(smach.State):

    def __init__(self, ac):
        smach.State.__init__(self, outcomes=['reached', 'aborted'])
        self.ac = ac
    def execute(self, userdata):
        if not self.ac.aborted:
            rospy.loginfo("Using predicted values to find pinger location")
            self.estimated = self.ac.getEstimated(self.ac.PointsCollected)
            self.ac.navClient(self.estimated[0], self.estimated[1])
            self.ac.taskComplete()
            return 'reached'
        else:
            return 'aborted'

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
    global conn
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

class Test(smach.State):
    global sock
    def __init__(self,ac):
        smach.State.__init__(self, outcomes=['completed'])
        self.ac = ac
        self.pingerLocation = [self.ac.pos['x'], self.ac.pos['y']] 
    def execute(self,userdata):
        self.ac.sendMovement(f = -3.0)
        data = conn.recv(BUFFER_SIZE)
        while not self.ac.aborted or not self.ac.killed:
            self.ac.logger(data, self.pingerLocation, LOGFILE)
        return 'completed'
        
        

#Class functions 
def quitProgram():
    global isKilled
    isKilled = True

def connectToLabView(obj):
    global sock, conn
    while not obj.killed:
        (conn, addr) = sock.accept()	
    conn.close()

def main():
    global sock, ros_param
    #Check if killed
    signal.signal(signal.SIGINT, quitProgram)
    #Initialise socket connection
    sock.bind((TCP_IP, TCP_PORT))
    sock.listen(1)
    connectToLabView(acoustic)
    

    #Publish labview data
    while not acoustic.aborted:
        acoustic.angle_pub.publish(acoustic.normaliseHeading(conn.recv(BUFFER_SIZE), acoustic.heading['yaw'])) 

    #Creating a State Machine Container
    sm = smach.StateMachine(outcomes=['found_pinger', 'aborted']) 

    with sm:
        smach.StateMachine.add("DISENGAGE", Disengage(acoustic),
                               transitions={'initialised':'PASSIVE_READING','test':'TEST', 'aborted':'aborted', 'eureka':'found_pinger', 'active':'ACTIVE_READING'})

        smach.StateMachine.add("USE_PREDICTED", UsePredicted(acoustic),
                               transitions={'reached':'found pinger',
                                            'aborted':'DISENGAGE'})

        smach.StateMachine.add("ACTIVE_READING", ActiveReading(acoustic),
                                transitions={'moving':'ACTIVE_READING',
                                             'failed':'USE_PREDICTED', 
                                             'identified_pinger':'DISENGAGE'})
        smach.StateMachine.add("PASSIVE_READING", PassiveReading(acoustic),
                                transitions={'completed':'ACTIVE_READING', 'failed':'DISENGAGE' })
        smach.StateMachine.add("TEST", Test(acoustic),
                               transitions={'completed':'DISENGAGE'})

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
