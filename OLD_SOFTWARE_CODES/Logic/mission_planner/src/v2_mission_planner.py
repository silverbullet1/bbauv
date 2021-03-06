#!/usr/bin/env python2

import roslib; roslib.load_manifest('mission_planner')
import rospy
from bbauv_msgs.srv import *
from bbauv_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import actionlib
import smach
import smach_ros
import bbauv_msgs

import bbauv_msgs.msg

class Countdown(smach.State):
    def __init__(self, sleep=2.0):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.sleep_time = sleep
    
    def execute(self, userdata):
        
        #This is to allow enough time for ethernet cable to be 
        rospy.loginfo("Going for COUNTDOWN")        
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time) < self.sleep_time:
            r.sleep()
        return 'succeeded'        

class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_complete'])
    def execute(self,userdata):
    
        global locomotionGoal
        global locomotion_client
        
        rospy.loginfo('Executing state START')
        
        #Reset DVL and Earth Odom here
        
        #Setting PID (Fwd? Side? Head? Depth? Pitch?) and modes (Topside? Nav?)
        rospy.wait_for_service('set_controller_srv')
        self.set_controller_request = rospy.ServiceProxy('set_controller_srv',set_controller)
        try:
            resp = self.set_controller_request(True, True, True, True, True, False, False)
            rospy.loginfo("PID and Mode is set")
        except rospy.ServiceException, e:
            rospy.loginfo("PID and Mode NOT set: %s" % e)
        
        #Key in starting position here
        goal = bbauv_msgs.msg.ControllerGoal()
        goal.forward_setpoint = 0
        goal.sidemove_setpoint = 0
        goal.depth_setpoint = locomotionGoal.depth_setpoint = 1.5
        goal.heading_setpoint = locomotionGoal.heading_setpoint = 65
        locomotion_client.send_goal(goal)
        locomotion_client.wait_for_result(rospy.Duration(30,0))
        rospy.loginfo("Dive Dive Dive!")
        
        return 'start_complete'

class Gate(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['gate_complete','gate_failed'])
        self.name = self.__class__.__name__
        
    def execute(self,userdata):
    
        global locomotionGoal
        global locomotion_client
        global movebase_client
        global isSearchDone
        global isTaskComplete
        isSearchDone = False
        isTaskComplete = False
    
        rospy.loginfo("Entering %s state" % self.name)
        
        #Connecting to task server
        rospy.wait_for_service('gate_srv')
        gate_srv = rospy.ServiceProxy('gate_srv', mission_to_vision)        
        rospy.loginfo('Mission Connected to %s Server' % self.name)
        
        #Setting PID (Fwd? Side? Head? Depth? Pitch?) and modes (Topside? Nav?)
        rospy.wait_for_service('set_controller_srv')
        self.set_controller_request = rospy.ServiceProxy('set_controller_srv',set_controller)
        try:
            resp = self.set_controller_request(True, True, True, True, True, False, False)
            rospy.loginfo("PID and Mode is set")
        except rospy.ServiceException, e:
            rospy.loginfo("PID and Mode NOT set: %s" % e)
        
        #Begin Searching For Task
        try:
            #gate_srv(start_request, start_ctrl, abort)
            resp = gate_srv(True, locomotionGoal, False)
            rospy.loginfo("Searching for %s" % self.name)
        except rospy.ServiceException, e:
            rospy.loginfo("Failed to start Search" % e)

        
        timeout = 120
        elapsed = 0
        search_status = False
        while(not rospy.is_shutdown()):
            t1 = rospy.Time.now()
            if elapsed <= timeout:
                if not isSearchDone:
                    #Moving around to search for Task
                    self.goal = bbauv_msgs.msg.ControllerGoal()
                    goal.forward_setpoint = 1
                    goal.sidemove_setpoint = 0
                    goal.heading_setpoint = locomotionGoal.heading_setpoint
                    goal.depth_setpoint = locomotionGoal.depth_setpoint
                    #locomotion_client.send_goal(self.goal)
                    rospy.loginfo("%s: Moving Fwd" % self.name)
                    #locomotion_client.wait_for_result()                      

                if isSearchDone:
                    #Turn off navigation mode if need be
                    rospy.loginfo("Found: Task Controlling Vehicle")
                        
                if isTaskComplete:
                    rospy.loginfo("Gate State Ended")
                    return 'gate_complete'
            else:
                #What to do when timeout?: For gate task, surface and re-run
                rospy.loginfo("Fail to find task")
                goal.forward_setpoint = 0
                goal.sidemove_setpoint = 0
                goal.depth_setpoint = 0
                resp = gate_srv(False, goal, True)
                rospy.loginfo("Unable to complete %s" % self.name)
                return 'gate_failed'
                
            t2 = rospy.Time.now()
            elapsed += (t2-t1).to_sec()
            rospy.loginfo("%s task: %d of %d secs elapsed" % (self.name, elapsed, timeout))
            
class Lane_Gate(smach.State):
    goal = None
    isSearchDone = False
    isTaskComplete = False
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['lane_complete'])
        self.name = self.__class__.__name__
                
    def execute(self,userdata):
    
        global locomotionGoal
        global locomotion_client
        global movebase_client
        global isSearchDone
        global isTaskComplete
        isSearchDone = False
        isTaskComplete = False
        
        #Service Server for each State
        mission_server = rospy.Service('mission_srv', vision_to_mission, self.handle_srv)
        rospy.loginfo('mission_srv for LANE_GATE initialized!')
        try:
            resp = lane_srv(True,locomotionGoal,True,2,False)
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

class Park(smach.State):    
    def __init__(self):
        smach.State.__init__(self, outcomes=['park_complete','park_failed'])
        self.name = self.__class__.__name__
    def execute(self,userdata):
    
        global locomotionGoal
        global locomotion_client
        global movebase_client
        global isSearchDone
        global isTaskComplete
        isSearchDone = False
        isTaskComplete = False
        goal = bbauv_msgs.msg.ControllerGoal()
        rospy.loginfo("Entering %s state" % self.name)
        
        #Connecting to task server
        rospy.wait_for_service('park_srv')
        park_srv = rospy.ServiceProxy('park_srv', mission_to_vision)        
        rospy.loginfo('Mission Connected to %s Server' % self.name)
        
        #Setting PID (Fwd? Side? Head? Depth? Pitch?) and modes (Topside? Nav?)
        rospy.wait_for_service('set_controller_srv')
        self.set_controller_request = rospy.ServiceProxy('set_controller_srv',set_controller)
        try:
            resp = self.set_controller_request(True, True, True, True, True, False, False)
            rospy.loginfo("PID and Mode is set")
        except rospy.ServiceException, e:
            rospy.loginfo("PID and Mode NOT set: %s" % e)
        
        #Begin Searching For Task
        try:
            #gate_srv(start_request, start_ctrl, abort)
            resp = park_srv(True, locomotionGoal, False)
            rospy.loginfo("Searching for %s" % self.name)
        except rospy.ServiceException, e:
            rospy.loginfo("Failed to start Search" % e)
        
        task_timeout = 180
        search_timeout = 60
        elapsed = 0
        counter = True 
        while(not rospy.is_shutdown()):
            t1 = rospy.Time.now()
            if elapsed <= task_timeout:
                if not isSearchDone:
                    #Moving around to search for Task
                    goal.forward_setpoint = 1
                    goal.sidemove_setpoint = 0
                    goal.heading_setpoint = locomotionGoal.heading_setpoint
                    goal.depth_setpoint = locomotionGoal.depth_setpoint
                    locomotion_client.send_goal(goal)
                    rospy.loginfo("%s: Moving Fwd. %d of %d secs elapsed" % (self.name, elapsed, task_timeout))
                    locomotion_client.wait_for_result()                      

                if isSearchDone and counter:
                    #Turn off navigation mode if need be
                    #Seed goal                     
                    locomotionGoal.depth_setpoint = goal.depth_setpoint
                    locomotionGoal.heading_setpoint = goal.heading_setpoint
                    try:
                        resp = park_srv(True, locomotionGoal, False)              
                    except rospy.ServiceException, e:
                        rospy.loginfo("Failed to seed goal" % e)
                       
                    rospy.loginfo("^%s: Found. Task Controlling Vehicle. %d of %d secs elapsed" % (self.name, elapsed, task_timeout))
                    counter = False
                        
                if isTaskComplete:
                    rospy.loginfo("%s: State Ended. %d of %d secs elapsed" % (self.name, elapsed, task_timeout))
                    return 'park_complete'
            if elapsed > task_timeout:
                #What to do when timeout?: For gate task, surface and re-run
                rospy.loginfo("%s: Fail to find task" % self.name)
                locomotionGoal.depth_setpoint = 0
                resp = park_srv(False, locomotionGoal, True)
                rospy.loginfo("%s: Unable to complete. %d of %d secs elapsed" % (self.name, elapsed, task_timeout))
                return 'park_failed'
                
            t2 = rospy.Time.now()
            elapsed += (t2-t1).to_sec()
    
class CreepSearch(smach.state):
    def __init__(self, task_name, timeout, start_heading=locomotionGoal.heading_setpoint, start_depth=locomotionGoal.heading_setpoint):
        smach.State.__init__(self, outcomes=['creep_complete','failed'])
        self.name = self.__class__.__name__  
        self.task_srv_name = task_name + 'srv'
        self.task_srv = None
                   
    def execute(self, userdata):
    
        global locomotionGoal
        global locomotion_client
        global isSearchDone
        isSearchDone = False
        
        goal = bbauv_msgs.msg.ControllerGoal()
        rospy.loginfo("Entering %s %s state" % (task_name, self.name))  
        
        #connecting to task server
        rospy.wait_for_service(self.task_srv_name)   
        self.task_srv = rospy.ServiceProxy(self.task_srv_name, mission_to_vision)
        rospy.loginfo('Mission Connected to %s Server' % task_name)
        
        #Setting PID (Fwd? Side? Head? Depth? Pitch?) and modes (Topside? Nav?)
        rospy.wait_for_service('set_controller_srv')
        self.set_controller_request = rospy.ServiceProxy('set_controller_srv',set_controller)
        try:
            resp = self.set_controller_request(True, True, True, True, True, False, False)
            rospy.loginfo("PID and Mode is set")
        except rospy.ServiceException, e:
            rospy.loginfo("PID and Mode NOT set: %s" % e)
        
        #Begin Searching For Task
        goal.heading_setpoint = locomotionGoal.heading_setpoint = start_heading
        goal.depth_setpoint = locomotionGoal.depth_setpoint = start_depth
        try:
            resp = self.task_srv(True, locomotionGoal, False)
            rospy.loginfo("Searching for %s" % task_name)
        except rospy.ServiceException, e:
            rospy.loginfo("Failed to start Search" % e)  
        
        #Performing Creep Search
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        while (not rospy.is_shutdown) and ((rospy.get_time()-start_time) <= timeout):
            if not isSearchDone:
                goal.forward_setpoint = 1
                goal.sidemove_setpoint = 0
                locomotion_client.send_goal(goal)
                rospy.loginfo("Moving Fwd to %s. %d of %d secs elapsed" % (task_name, rospy.get_time()-start_time, timeout))                
                locomotion_client.wait_for_result()                      
            if isSearchDone:                
                return 'creep_complete'
            r.sleep()
            
        try:
            rospy.loginfo('Failed to find %s' % task_name)
            resp = self.task_srv(False, None, True)            
            return 'failed'              
        except rospy.ServiceException, e:
            rospy.loginfo("Failed to seed goal" % e)
        return 'failed'
        
class WaitOut(smach.state):
    def __init__(self, task_name, timeout):
        smach.State.__init__(self, outcomes=['task_complete','failed'])
        self.name = self.__class__.__name__ 
        self.task_srv_name = task_name + 'srv'
        self.task_srv = None         
        
    def execute(self, userdata):    
        global isTaskComplete
        isTaskComplete = False        
        rospy.loginfo("Entering %s %s state" % (task_name, self.name))  
        
        #connecting to task server
        rospy.wait_for_service(self.task_srv_name)   
        self.task_srv = rospy.ServiceProxy(self.task_srv_name, mission_to_vision)
        rospy.loginfo('Mission Connected to %s Server' % task_name)        
        
        #Waiting Out
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        while (not rospy.is_shutdown) and ((rospy.get_time()-start_time) <= timeout):
                
            if isTaskComplete:
                rospy.loginfo("Completed %s. %d of %d secs elapsed" % (task_name, rospy.get_time()-start_time, timeout)) 
                return 'task_complete'
            r.sleep()
            
        try:
            rospy.loginfo('Failed to complete %s' % task_name)
            resp = self.task_srv(False, None, True)            
            return 'failed'              
        except rospy.ServiceException, e:
            rospy.loginfo("Failed to seed goal" % e)
        return 'failed'                    
     


def handle_srv(req):
    global locomotionGoal
    global isSearchDone
    global isTaskComplete
    
    #Search completion request from Vision Node.
    if(req.search_request):
        isSearchDone = True
        rospy.loginfo("Search complete")
        return vision_to_missionResponse(True,False)
    
    #Task completion request from Vision Node.
    if(req.task_complete_request):
        rospy.loginfo("Task complete")
        locomotionGoal = req.task_complete_ctrl 
        isTaskComplete = True
        #Controller
        return vision_to_missionResponse(False,True)   

locomotion_client = None
locomotionGoal = controller()
movebase_client = None
movebaseGoal = None

mission_server = None
isSearchDone = False
isTaskComplete = False
lane_srv = None

if __name__ == '__main__':
    rospy.init_node('Mission_planner', anonymous=True)
    
    mission_server = rospy.Service('mission_srv', vision_to_mission, handle_srv)
    rospy.loginfo('MissionServer Initialized!')
    
    #Service Client for Lane; Lane task is the only task that will not be shutdown
#    rospy.loginfo('Waiting for LaneServer to start up...')
#    rospy.wait_for_service('lane_srv')
#    lane_srv = rospy.ServiceProxy('lane_srv', mission_to_lane)
#    rospy.loginfo('Mission Connected to LaneServer')
    
    # Action Client for PIDs
    locomotion_client = actionlib.SimpleActionClient('LocomotionServer', bbauv_msgs.msg.ControllerAction)
    locomotion_client.wait_for_server()
    rospy.loginfo("Mission connected to LocomotionServer")
    
    # Action Client for Move Base
#    movebase_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
#    movebase_client.wait_for_server()
    rospy.loginfo("Mission connected to MovebaseServer")
    movebaseGoal = MoveBaseGoal()
    
    sm_mission = smach.StateMachine(outcomes=['mission_complete','mission_failed'])
    
    with sm_mission:
        smach.StateMachine.add('COUNTDOWN', Countdown(), transitions={'succeeded':'START'})
        smach.StateMachine.add('START',Start(),
                                transitions={'start_complete':'GATE'})
        smach.StateMachine.add('GATE',Gate(),transitions={'gate_complete':'PARK','gate_failed':'mission_failed'})
        #smach.StateMachine.add('LANE_GATE',Lane_Gate(),transitions={'lane_complete':'mission_complete'})
        smach.StateMachine.add('PARK',Park(),transitions={'park_complete':'mission_complete','park_failed':'mission_failed'})
        
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('gate_server', sm_mission, '/MISSION')
    sis.start()
    try:
        outcome = sm_mission.execute()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
