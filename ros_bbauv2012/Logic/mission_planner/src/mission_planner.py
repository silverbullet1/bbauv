#!/usr/bin/env python2

import roslib; roslib.load_manifest('mission_planner')
import rospy
from bbauv_msgs.srv import *
from bbauv_msgs.msg import *

import actionlib
import smach
import smach_ros
import bbauv_msgs

import bbauv_msgs.msg

class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_complete'])
    def execute(self,userdata):
        #include countdown
        rospy.loginfo('Executing state START')
        return 'start_complete'

class Gate(smach.State):
    client = None
    goal = None
    isSearchDone = False
    isTaskComplete = False
    isAbort = False
    def __init__(self):
        smach.State.__init__(self, outcomes=['gate_complete'])
    
    def handle_search_node(self,req):
        self.isStart = True
        return sm_startResponse(self.isStart)
       
    def doneCB(self,status,result):
        
        #print result.forward_final;
        #print result.heading_final;
        #print result.sidemove_final;
        if status == actionlib.GoalStatus.SUCCEEDED:
            self.client.send_goal(self.goal,self.doneCB)
        elif status == actionlib.GoalStatus.PREEMPTED:
            rospy.loginfo(str(rospy.get_name()) + "Nav PREEMPTED.")
    
    def handle_srv(self,req):
        return vision_to_missionResponse(self.isStart)   
    def execute(self,userdata):
        rospy.loginfo('Executing state GATE')
        #s = rospy.Service('search_gate_node', sm_search, self.handle_search_node)
        
        self.client = actionlib.SimpleActionClient('LocomotionServer', bbauv_msgs.msg.ControllerAction)
        self.client.wait_for_server()
         # Creates a goal to send to the action server.to move the AUV forward
        self.goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=5,heading_setpoint=50,depth_setpoint=0.5,sidemove_setpoint=0)
        # Sends the goal to the action server.
        self.client.send_goal(self.goal,self.doneCB)
        rospy.loginfo("Goal sent. Vehicle moving forward towards Gate Task")
        #Service Server
        srvServer = rospy.Service('mission_srv', vision_to_mission, self.handle_srv)
        rospy.loginfo('mission_srv initialized!')
        
        try:
            ctrl = controller()
            ctrl.depth_setpoint = 0.5
            '''
                Service declaration
                ### Mission to Vision
                
                ### Vision to Mission
            
            '''
            
            #Service Client 
            rospy.loginfo('Waiting for gate_srv to start up...')
            rospy.wait_for_service('gate_srv')
            gate_srv = rospy.ServiceProxy('gate_srv', mission_to_vision)
            rospy.loginfo('connected to gate_srv!')
            # Format for service: start_request, start_ctrl, abort_request
            
            #Start up Gate Node
            rospy.loginfo('Starting up Gate Node.')
            resp = gate_srv(True,ctrl,None)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        while(not rospy.is_shutdown()):
            if self.isSearchDone:
                self.client.cancel_all_goals()
            if self.isAbort:
                pass
            if self.isTaskComplete:
                pass
            
        return 'gate_complete'

class Lane_Gate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['lane_complete'])
    def execute(self,userdata):
        rospy.loginfo('Executing state LANE_GATE')
        return 'lane_complete'
    
class Mission_planner():
    def __init__(self):
        pass
    
if __name__ == '__main__':
    rospy.init_node('Mission_planner', anonymous=True)
    mission_planner = Mission_planner()
    sm_mission = smach.StateMachine(outcomes=['mission_complete'])
    
    with sm_mission:
        smach.StateMachine.add('START',Start(),
                                transitions={'start_complete':'GATE'})
        smach.StateMachine.add('GATE',Gate(),transitions={'gate_complete':'LANE_GATE'})
        smach.StateMachine.add('LANE_GATE',Lane_Gate(),transitions={'lane_complete':'mission_complete'})
        
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('gate_server', sm_mission, '/MISSION')
    sis.start()
    try:
        outcome = sm_mission.execute()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
