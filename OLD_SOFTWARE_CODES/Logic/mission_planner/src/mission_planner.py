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
    isStart = False
    def __init__(self):
        smach.State.__init__(self, outcomes=['gate_complete'])
       
    def doneCB(self,status,result):
        
        #print result.forward_final;
        #print result.heading_final;
        #print result.sidemove_final;
        if status == actionlib.GoalStatus.SUCCEEDED and not self.isSearchDone:
            rospy.loginfo("moving vehicle forward 5m again")
            self.client.send_goal(self.goal,self.doneCB)
        elif status == actionlib.GoalStatus.PREEMPTED:
            rospy.loginfo(str(rospy.get_name()) + "Nav PREEMPTED.")
    
    def handle_srv(self,req):
        global locomotionGoal
        #Search completion request from Vision Node.
        if(req.search_request):
            self.isSearchDone = True
            rospy.loginfo("search complete")
            return vision_to_missionResponse(True,False)
        
        #Task completion request from Vision Node.
        if(req.task_complete_request):
            rospy.loginfo("Gate task complete")
            locomotionGoal = req.task_complete_ctrl 
            self.isTaskComplete = True
            #Controller
            return vision_to_missionResponse(False,True)   
        
    def execute(self,userdata):
        #Service Server for each State
        mission_server = rospy.Service('mission_srv', vision_to_mission, self.handle_srv)
        rospy.loginfo('mission_srv for GATE initialized!')
    
        rospy.wait_for_service('set_controller_srv')
        self.set_controller_request = rospy.ServiceProxy('set_controller_srv',set_controller)
        try:
            resp = self.set_controller_request(True, True, True, True, True, False)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        rospy.loginfo('Waiting for LocomotionServer Action Server')
        self.client = actionlib.SimpleActionClient('LocomotionServer', bbauv_msgs.msg.ControllerAction)
        self.client.wait_for_server()
         # Creates a goal to send to the action server.to move the AUV forward
        self.goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=5,heading_setpoint=55,depth_setpoint=0.6,sidemove_setpoint=0)
        # Sends the goal to the action server.
        self.client.send_goal(self.goal,self.doneCB)
        rospy.loginfo("Goal sent. Vehicle moving forward towards Gate Task")
       
        try:
            ctrl = controller()
            ctrl.depth_setpoint = 0.6
            ctrl.heading_setpoint = 55
            '''
                Service declaration
                ### Mission to Vision
                
                ### Vision to Mission
            
            '''
            #Service Client for Gate
            rospy.loginfo('Waiting for gate_srv to start up...')
            rospy.wait_for_service('gate_srv')
            gate_srv = rospy.ServiceProxy('gate_srv', mission_to_vision)
            
            rospy.loginfo('connected to gate_srv!')
            
            # Format for service: start_request, start_ctrl, abort_request
            
            #Start up Gate Node
            rospy.loginfo('Starting up Gate Node.')
            resp = gate_srv(True,ctrl,False)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        while(not rospy.is_shutdown()):
            if self.isSearchDone:
                #self.client.cancel_all_goals()
                pass
            if self.isAbort:
                pass
            if self.isTaskComplete:
                mission_server.shutdown("Gate State ended")
                return 'gate_complete'
                pass

class Lane_Gate(smach.State):
    isSearchDone = False
    isTaskComplete = False
    def __init__(self):
        smach.State.__init__(self, outcomes=['lane_complete'])
    def handle_srv(self,req):
        global locomotionGoal
        #Search completion request from Vision Node.
        if(req.search_request):
            self.isSearchDone = True
            rospy.loginfo("lane search complete")
            return vision_to_missionResponse(True,False)
        
        #Task completion request from Vision Node.
        if(req.task_complete_request):
            rospy.loginfo("lane task complete")
            locomotionGoal = req.task_complete_ctrl 
            self.isTaskComplete = True
            #Controller
            return vision_to_missionResponse(False,True)   
        
    def execute(self,userdata):
        global locomotionGoal
        global lane_srv
        global movement_client
        #Service Server for each State
        mission_server = rospy.Service('mission_srv', vision_to_mission, self.handle_srv)
        rospy.loginfo('mission_srv for LANE_GATE initialized!')
        try:
            resp = lane_srv(True,locomotionGoal,False,2,False)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        #Start NAV Module
        while not rospy.is_shutdown():
            while not rospy.is_shutdown() and not self.isSearchDone:
                 goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=3,heading_setpoint=locomotionGoal.heading_setpoint,depth_setpoint=locomotionGoal.depth_setpoint,sidemove_setpoint=locomotionGoal.sidemove_setpoint)
                 movement_client.send_goal(goal)
                 movement_client.wait_for_result()
            
            while not rospy.is_shutdown() and self.isTaskComplete: 
                goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0,heading_setpoint=locomotionGoal.heading_setpoint,depth_setpoint=locomotionGoal.depth_setpoint,sidemove_setpoint=0)
                movement_client.send_goal(goal)
                movement_client.wait_for_result()
                goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=3,heading_setpoint=locomotionGoal.heading_setpoint,depth_setpoint=locomotionGoal.depth_setpoint,sidemove_setpoint=0)
                movement_client.send_goal(goal)
                movement_client.wait_for_result()
                return 'lane_complete'
    
class Mission_planner():
    def __init__(self):
        pass

movement_client = None
mission_server = None
lane_srv = None
locomotionGoal = None
if __name__ == '__main__':
    rospy.init_node('Mission_planner', anonymous=False)
    
    #Service Client for Lane
    rospy.loginfo('Waiting for lane_srv to start up...')
    rospy.wait_for_service('lane_srv')
    lane_srv = rospy.ServiceProxy('lane_srv', mission_to_lane)
    rospy.loginfo('connected to lane_srv!')
    
    movement_client = actionlib.SimpleActionClient('LocomotionServer', bbauv_msgs.msg.ControllerAction)
    movement_client.wait_for_server()
        
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
