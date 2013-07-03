#!/usr/bin/env python2

import roslib; roslib.load_manifest('mission_planner')
import rospy
from bbauv_msgs.srv import *
from bbauv_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler, quaternion_about_axis
import dynamic_reconfigure.client
from math import pi
import subprocess

import actionlib
import smach
import smach_ros
import bbauv_msgs
import bbauv_msgs.msg
from nav_msgs.msg import Odometry
from bbauv_msgs.msg import depth
from bbauv_msgs.msg import imu_data

class Countdown(smach.State):
    def __init__(self, sleep=1.0):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.sleep_time = sleep
    
    def execute(self, userdata):
        
        #This is to allow enough time for ethernet cable to be removed
        rospy.loginfo("Going for COUNTDOWN")        
        r = rospy.Rate(1)
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time) < self.sleep_time:
            r.sleep()
            rospy.loginfo("%d Elapsed" % (rospy.get_time() - start_time))
        return 'succeeded'        

class Start(smach.State):
    def __init__(self, timeout, start_depth, start_heading):
        smach.State.__init__(self, outcomes=['start_complete'])
        self.name = self.__class__.__name__
        self.timeout = timeout                
        self.start_depth = start_depth
        self.start_heading = start_heading        
                
    def execute(self,userdata):
#        return 'start_complete'
        global locomotionGoal
        global locomotion_client
        global set_ConPIDMode
        rospy.loginfo('Executing state START')
        
        #Key in starting position here
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0, sidemove_setpoint=0,
                                            depth_setpoint= self.start_depth, 
                                            heading_setpoint = self.start_heading                                            
                                            )

        #Reset DVL and Earth Odom here
        drClient_DVL = dynamic_reconfigure.client.Client("WH_DVL")
        drClient_DVL.update_configuration({"zero_distance":True})
        drClient_Earth = dynamic_reconfigure.client.Client("earth_odom")
        drClient_Earth.update_configuration({"zero_distance":True})        

        rospy.sleep(2)  
      
        #Setting PID (Fwd? Side? Head? Depth? Pitch?) and modes (Topside? Nav?)
        try:
            resp = set_ConPIDMode(True, True, True, True, False, False, False)
            rospy.loginfo("PID and Mode is set")
        except rospy.ServiceException, e:
            rospy.loginfo("PID and Mode NOT set: %s" % e)

        locomotion_client.send_goal(goal)
        locomotion_client.wait_for_result(rospy.Duration(self.timeout,0))
        rospy.loginfo("Dive Dive Dive!")

        
        locomotionGoal.depth_setpoint = self.start_depth
        locomotionGoal.heading_setpoint = self.start_heading
        locomotionGoal.sidemove_setpoint = 0
        locomotionGoal.forward_setpoint = 0
        return 'start_complete'

class GoToDistance(smach.State):
    def __init__(self, timeout, distance, direction):
        smach.State.__init__(self, outcomes=['distance_complete'])
        self.distance = distance
        self.timeout = timeout
        self.direction = direction
        
    def execute(self,userdata):
        global locomotionGoal
        global locomotion_client
        
        if self.direction == 'fwd':
            goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=self.distance,
                                                sidemove_setpoint=0,
                                                depth_setpoint=locomotionGoal.depth_setpoint,
                                                heading_setpoint=locomotionGoal.heading_setpoint)
        
        if self.direction == 'sway':
            goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0,
                                                sidemove_setpoint=self.distance,
                                                depth_setpoint=locomotionGoal.depth_setpoint,
                                                heading_setpoint=locomotionGoal.heading_setpoint)

        rospy.loginfo('Going %s distance %s' % (str(self.direction), str(self.distance)))                                            
        locomotion_client.send_goal(goal)
        locomotion_client.wait_for_result(rospy.Duration(self.timeout,0))
             
        return 'distance_complete'

class GoToDepth(smach.State):
    def __init__(self, timeout=3, depth=None, surface=False):
        smach.State.__init__(self, outcomes=['depth_complete'])
        self.depth = depth
        self.timeout = timeout
        self.surface = surface
        
    def execute(self,userdata):
        global locomotionGoal
        global locomotion_client

        if self.depth == None:
            if locomotionGoal.depth_setpoint > 0 :
                self.depth = locomotionGoal.depth_setpoint
            if locomotionGoal.depth_setpoint <=0:
                if self.surface == True:
                    self.depth = locomotionGoal.depth_setpoint
                if self.surface == False:
                    self.depth = 0.5
                    rospy.loginfo('Trying to surface PREMATURELY! Depth constrained to 0.5 meters')
        
        #seed goal
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0,
                                            sidemove_setpoint=0,
                                            depth_setpoint=self.depth,
                                            heading_setpoint=locomotionGoal.heading_setpoint)
        rospy.loginfo('Going to depth %s' % str(self.depth))
        locomotion_client.send_goal(goal)
        locomotion_client.wait_for_result(rospy.Duration(self.timeout,0))
        
        #Updating locomotionGoal
        locomotionGoal.depth_setpoint = goal.depth_setpoint        
        return 'depth_complete'

class GoToHeading(smach.State):
    def __init__(self, timeout=3, heading=None, relative=False):
        smach.State.__init__(self, outcomes=['heading_complete'])
        self.heading = heading
        self.relative = relative
        self.timeout = timeout

    def normalize_angle(self, angle):
        normalized = (angle%360+360)%360
        return normalized
        
    def execute(self,userdata):
        global locomotionGoal
        global locomotion_client
        
        if self.heading == None:
            self.heading = locomotionGoal.heading_setpoint
        
        if self.heading != None and self.relative == True:
            self.heading = self.normalize_angle(self.heading+locomotionGoal.heading_setpoint)

        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0,
                                            sidemove_setpoint=0,
                                            depth_setpoint=locomotionGoal.depth_setpoint,
                                            heading_setpoint=self.heading)

        rospy.loginfo('Going to heading %s' % str(self.heading))                                            
        locomotion_client.send_goal(goal)
        locomotion_client.wait_for_result(rospy.Duration(self.timeout,0))
        
        #Updating locomotionGoal
        locomotionGoal.heading_setpoint = goal.heading_setpoint      
        return 'heading_complete'

    
class StoreGlobalCoord(smach.State):
    def __init__(self, task_name):
        smach.State.__init__(self, outcomes=['store_complete'])
        self.name = self.__class__.__name__ 
        self.task_name = task_name
            
    def execute(self, userdata):
        global global_x
        global global_y
        global global_depth
        global global_heading

        rospy.loginfo('x=%s y=%s depth=%s heading=%s' % (str(global_x), str(global_y), str(global_depth), str(global_heading)))
        rospy.set_param(self.task_name+'/x', global_x)
        rospy.set_param(self.task_name+'/y', global_y)
        rospy.set_param(self.task_name+'/depth', global_depth)
        rospy.set_param(self.task_name+'/heading', global_heading)
            
        return 'store_complete'
        
class HoverSearch(smach.State):
    def __init__(self, task_name, timeout, use_left=False, num_lanes=0, start_depth=None, start_heading=None):
        smach.State.__init__(self, outcomes=['hover_complete', 'failed'])
        self.name = self.__class__.__name__
        self.task_name = task_name
        self.timeout = timeout
        self.use_left = use_left
        self.num_lanes = num_lanes
        self.start_depth = start_depth
        self.start_heading = start_heading
        
    def execute(self, userdata):
        global locomotionGoal
        global locomotion_client
        global set_ConPIDMode
        global lane_srv
        global isSearchDone
        isSearchDone = False

        rospy.loginfo("Entering %s %s state" % (self.task_name, self.name))
        
        #Setting PID (Fwd? Side? Head? Depth? Pitch?) and modes (Topside? Nav?)
        try:
            resp = set_ConPIDMode(True, True, True, True, False, False, False)
            rospy.loginfo("PID and Mode is set")
        except rospy.ServiceException, e:
            rospy.loginfo("PID and Mode NOT set: %s" % e)
            return 'failed'        

        #connecting to task server            
        if self.task_name != 'lane':
            rospy.wait_for_service(self.task_srv_name)   
            self.task_srv = rospy.ServiceProxy(self.task_srv_name, mission_to_vision)
            rospy.loginfo('Mission Connected to %s Server' % self.task_name)

        #Seeding goal
        goal = bbauv_msgs.msg.ControllerGoal()
        if self.start_heading == None: goal.heading_setpoint = locomotionGoal.heading_setpoint
        if self.start_heading != None: goal.heading_setpoint = locomotionGoal.heading_setpoint = self.start_heading
        if self.start_depth == None: goal.depth_setpoint = locomotionGoal.depth_setpoint
        if self.start_depth != None: goal.depth_setpoint = locomotionGoal.depth_setpoint = self.start_depth
        rospy.logdebug("start_heading = %s, start_depth = %s" % (str(goal.heading_setpoint), str(goal.depth_setpoint)))
        goal.forward_setpoint = 0
        goal.sidemove_setpoint = 0

        #Begin Hovering and Searching For Task
        if self.task_name == 'lane':
            try:
                resp = lane_srv(True,locomotionGoal,self.use_left,self.num_lanes,False)
            except rospy.ServiceException, e:
                rospy.loginfo("Failed to start Search" % e)
                return 'failed'  
        if self.task_name != 'lane':
            try:
                resp = self.task_srv(True, locomotionGoal, False)
                rospy.loginfo("Searching for %s" % self.task_name)
            except rospy.ServiceException, e:
                rospy.loginfo("Failed to start Search" % e)
                return 'failed'

        r = rospy.Rate(30)
        start_time = rospy.get_time()
        rospy.loginfo("Hovering to search for %s" % self.task_name)
        while (not rospy.is_shutdown()) and ((rospy.get_time()-start_time) <= self.timeout):
            if not isSearchDone:
                locomotion_client.send_goal(goal)
                locomotion_client.wait_for_result(rospy.Duration(1,0))
                                    
            if isSearchDone:                
                rospy.loginfo("Found %s. %d of %d secs elapsed" % (self.task_name, rospy.get_time()-start_time, self.timeout))
                return 'hover_complete'
            r.sleep()
        
        #Timed out
        if self.task_name == 'lane':
            try:
                rospy.loginfo('Failed to find %s' % self.task_name)
                resp = lane_srv(False, locomotionGoal, False, 1, True)            
                return 'failed'              
            except rospy.ServiceException, e:
                rospy.loginfo("Failed to abort: %s" % e)    
                return 'failed'                         
        if self.task_name != 'lane':
            try:
                rospy.loginfo('Failed to find %s' % self.task_name)
                resp = self.task_srv(False, locomotionGoal, True)            
                return 'failed'              
            except rospy.ServiceException, e:
                rospy.loginfo("Failed to abort: %s" % e)
                return 'failed'                   
        
        return 'hover_complete'        
        
class LinearSearch(smach.State):
    def __init__(self, task_name, timeout, distance, direction, use_left=False, num_lanes=0, start_depth=None, start_heading=None):
        smach.State.__init__(self, outcomes=['linear_complete','failed'])
        self.name = self.__class__.__name__
        self.task_name = task_name
        self.timeout = timeout        
        self.task_srv_name = task_name + '_srv'
        self.task_srv = None
        self.distance = distance
        self.direction = direction
        self.use_left = use_left
        self.num_lanes = num_lanes
        self.start_depth = start_depth
        self.start_heading = start_heading
        
        self.motionStatus = False

    def motion_callback(self,status,result):
        if status==actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Search Callback Motion Complete")
            self.motionStatus = True
                   
    def execute(self, userdata):
#        return 'linear_complete'
        global locomotionGoal
        global locomotion_client
        global set_ConPIDMode
        global lane_srv
        global isSearchDone
        isSearchDone = False
        
        rospy.loginfo("Entering %s %s state" % (self.task_name, self.name))  
        

        #Setting PID (Fwd? Side? Head? Depth? Pitch?) and modes (Topside? Nav?)
        try:
            resp = set_ConPIDMode(True, True, True, True, False, False, False)
            rospy.loginfo("PID and Mode is set")
        except rospy.ServiceException, e:
            rospy.loginfo("PID and Mode NOT set: %s" % e)
            return 'failed'
        
        #connecting to task server            
        if self.task_name != 'lane':
            rospy.wait_for_service(self.task_srv_name)   
            self.task_srv = rospy.ServiceProxy(self.task_srv_name, mission_to_vision)
            rospy.loginfo('Mission Connected to %s Server' % self.task_name)
        
        #Seeding goal
        goal = bbauv_msgs.msg.ControllerGoal()
        if self.start_heading == None: goal.heading_setpoint = locomotionGoal.heading_setpoint
        if self.start_heading != None: goal.heading_setpoint = locomotionGoal.heading_setpoint = self.start_heading
        if self.start_depth == None: goal.depth_setpoint = locomotionGoal.depth_setpoint
        if self.start_depth != None: goal.depth_setpoint = locomotionGoal.depth_setpoint = self.start_depth
        rospy.logdebug("start_heading = %s, start_depth = %s" % (str(goal.heading_setpoint), str(goal.depth_setpoint)))
                
        #Begin Searching For Task
        if self.task_name == 'lane':
            try:
                resp = lane_srv(True,locomotionGoal,self.use_left,self.num_lanes,False)
            except rospy.ServiceException, e:
                rospy.loginfo("Failed to start Search" % e)
                return 'failed'  
        if self.task_name != 'lane':
            try:
                resp = self.task_srv(True, locomotionGoal, False)
                rospy.loginfo("Searching for %s" % self.task_name)
            except rospy.ServiceException, e:
                rospy.loginfo("Failed to start Search" % e)
                return 'failed'  
        
        #Begin Moving Around while searching
        if self.direction == 'fwd':
            goal.forward_setpoint = self.distance
            goal.sidemove_setpoint = 0
            locomotion_client.send_goal(goal, self.motion_callback)
   
        if self.direction == 'sway':
            goal.forward_setpoint = 0
            goal.sidemove_setpoint = self.distance
            locomotion_client.send_goal(goal, self.motion_callback)    
        
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        rospy.loginfo("Moving to search for %s" % self.task_name)
        while (not rospy.is_shutdown()) and ((rospy.get_time()-start_time) <= self.timeout):
            if not isSearchDone:
                if self.motionStatus:
                    return 'failed'
                                    
            if isSearchDone:
                locomotion_client.cancel_all_goals()             
                rospy.loginfo("Found %s. %d of %d secs elapsed" % (self.task_name, rospy.get_time()-start_time, self.timeout))
                return 'linear_complete'
            r.sleep()
        
        #Timed out
        if self.task_name == 'lane':
            try:
                rospy.loginfo('Failed to find %s' % self.task_name)
                resp = lane_srv(False, locomotionGoal, False, 1, True)            
                return 'failed'              
            except rospy.ServiceException, e:
                rospy.loginfo("Failed to abort: %s" % e)    
                return 'failed'                         
        if self.task_name != 'lane':
            try:
                rospy.loginfo('Failed to find %s' % self.task_name)
                resp = self.task_srv(False, locomotionGoal, True)            
                return 'failed'              
            except rospy.ServiceException, e:
                rospy.loginfo("Failed to abort: %s" % e)
                return 'failed'                 
        
class WaitOut(smach.State):
    def __init__(self, task_name, timeout):
        smach.State.__init__(self, outcomes=['task_complete','failed'])
        self.name = self.__class__.__name__ 
        self.task_name = task_name
        self.task_srv_name = task_name + '_srv'
        self.task_srv = None
        self.timeout = timeout    
        
    def execute(self, userdata):
        global lane_srv
        global isTaskComplete
        global locomotion_client
        global locomotionGoal
        isTaskComplete = False        
        rospy.loginfo("Entering %s %s state" % (self.task_name, self.name))  

        #connecting to task server;      
        if self.task_name != 'lane':        
            #connecting to task server
            rospy.wait_for_service(self.task_srv_name)   
            self.task_srv = rospy.ServiceProxy(self.task_srv_name, mission_to_vision)
            rospy.loginfo('Mission Connected to %s Server' % self.task_name)        
        
        #Waiting Out
        r = rospy.Rate(0.5)
        start_time = rospy.get_time()
        rospy.loginfo("%s: Found. Task Controlling Vehicle" % (self.task_name))
        while (not rospy.is_shutdown()) and ((rospy.get_time()-start_time) <= self.timeout):
            if isTaskComplete:
                rospy.loginfo("Completed %s. %d of %d secs elapsed" % (self.task_name, rospy.get_time()-start_time, self.timeout))
                return 'task_complete'
            r.sleep()
            
        if self.task_name == 'lane':
            try:
                rospy.loginfo('Failed to complete %s' % self.task_name)
                resp = lane_srv(False, locomotionGoal, False, 1, True)            
                return 'failed'              
            except rospy.ServiceException, e:
                rospy.loginfo("Failed to abort: %s" % e)
                return 'failed'                              
     
        if self.task_name != 'lane':
            try:
                rospy.loginfo('Failed to complete %s' % self.task_name)
                resp = self.task_srv(False, locomotionGoal, True)            
                return 'failed'              
            except rospy.ServiceException, e:
                rospy.loginfo("Failed to abort: %s" % e)            
                return 'failed'                    

class NavMoveBase(smach.State):
    def __init__ (self, prep_timeout, nav_timeout, x=0, y=0, depth = 0.5, start_heading=0, place= None): #yaw here is BBAUV's controller convention'
        smach.State.__init__(self, outcomes=['nav_complete','failed'])
        self.name = self.__class__.__name__
        self.x = x
        self.y = y
        self.start_heading = start_heading
        self.depth = depth
        self.place = place
        self.prep_timeout = prep_timeout    
        self.nav_timeout = nav_timeout    

    def normalize_angle(self, angle):
        normalized = (angle%360+360)%360
        return normalized
        
    def execute (self, userdata):
        global movebase_client
        global locomotion_client
        global locomotionGoal
        global set_ConPIDMode
        movebaseGoal = MoveBaseGoal()
        
        #Check is need to get cooridnate from param server; if place is not None but a string, use string to query param server for nav coords; if place is None, use x,y and yaw            
        if self.place != None:        
            self.x = rospy.get_param(self.place + '/x')
            self.y = rospy.get_param(self.place + '/y')
            self.depth = rospy.get_param(self.place + '/depth')
            self.start_heading = rospy.get_param(self.place + '/heading')

        #Change Depth and Face Heading First
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0,sidemove_setpoint=0,depth_setpoint=self.depth,heading_setpoint=self.start_heading)

        locomotion_client.send_goal(goal)
        rospy.loginfo('Going to depth %s m. Facing yaw=%s deg' % (str(self.depth), str(self.start_heading)))
        locomotion_client.wait_for_result(rospy.Duration(self.prep_timeout,0))

        #convert yaw to move_base convention
        ros_heading = self.normalize_angle((360-(self.start_heading))) * (pi/180) #self.start_heading * (pi/180) 
        x,y,z,w = quaternion_from_euler(0,0,ros_heading) #input must be radians
        rospy.logdebug('z= %s w= %s' % (str(z),str(w)))

        #Setting PID (Fwd? Side? Head? Depth? Pitch?) and modes (Topside? Nav?)
        try:
            resp = set_ConPIDMode(True, True, True, True, False, False, True)
            rospy.loginfo("PID and Mode is set")
        except rospy.ServiceException, e:
            rospy.loginfo("PID and Mode NOT set: %s" % e)
                        
        #Execute Nav
        movebaseGoal.target_pose.header.frame_id = 'map'
        movebaseGoal.target_pose.header.stamp = rospy.Time.now()
        movebaseGoal.target_pose.pose.position.x = self.x
        movebaseGoal.target_pose.pose.position.y = self.y * -1
        movebaseGoal.target_pose.pose.orientation.x = 0
        movebaseGoal.target_pose.pose.orientation.y = 0
        movebaseGoal.target_pose.pose.orientation.z = z
        movebaseGoal.target_pose.pose.orientation.w = w
        
        movebase_client.send_goal(movebaseGoal)
        rospy.loginfo('Going to x=%s y=%s. Facing yaw=%s' % (str(self.x), str(self.y), str(self.start_heading)))
        movebase_client.wait_for_result(rospy.Duration(self.nav_timeout,0))

        #Setting PID (Fwd? Side? Head? Depth? Pitch?) and modes (Topside? Nav?)
        try:
            resp = set_ConPIDMode(True, True, True, True, False, False, False)
            rospy.loginfo("PID and Mode is set")
        except rospy.ServiceException, e:
            rospy.loginfo("PID and Mode NOT set: %s" % e)
            
        goal.depth_setpoint = self.depth
        goal.heading_setpoint = self.start_heading
        locomotion_client.send_goal(goal)
        rospy.loginfo('Going to depth %s. Facing yaw=%s' % (str(self.depth), str(self.start_heading)))
        locomotion_client.wait_for_result(rospy.Duration(self.prep_timeout,0))
        rospy.loginfo('Reached depth %s. Facing yaw=%s' % (str(self.depth), str(self.start_heading)))
        
        locomotionGoal.depth_setpoint = self.depth
        locomotionGoal.heading_setpoint = self.start_heading
        return 'nav_complete'
     
def handle_srv(req):
    global locomotionGoal
    global isSearchDone
    global isTaskComplete
    
    #Search completion request from Vision Node.
    if(req.search_request):
        isSearchDone = True
        rospy.logdebug("Search complete")
        return vision_to_missionResponse(True,False)
    
    #Task completion request from Vision Node.
    if(req.task_complete_request):
        rospy.logdebug("Task complete")
        locomotionGoal = req.task_complete_ctrl 
        isTaskComplete = True
        #Controller
        return vision_to_missionResponse(False,True)   

def odomCallback(msg):
    global global_x
    global global_y
    global_x =  msg.pose.pose.position.x
    global_y = msg.pose.pose.position.y
            
def depthCallback(msg):
    global global_depth
    global_depth = msg.depth
    
def AHRSCallback(msg):
    global global_heading
    global_heading = msg.orientation.z * (180/pi)

locomotion_client = None
locomotionGoal = controller()
movebase_client = None
set_ConPIDMode = None

mission_server = None
isSearchDone = False
isTaskComplete = False
lane_srv = None

global_x = 0
global_y = 0
global_heading = 0
global_depth = 0.3

if __name__ == '__main__':
    rospy.init_node('Mission_planner', anonymous=True)
    
    mission_server = rospy.Service('mission_srv', vision_to_mission, handle_srv)
    rospy.loginfo('MissionServer Initialized!')

    # Subscribing to Telemetry
    odom_sub = rospy.Subscriber('/earth_odom', Odometry, odomCallback)
    depth_sub = rospy.Subscriber('/depth', depth, depthCallback)
    AHRS_sub = rospy.Subscriber('/AHRS8_data_e', imu_data, AHRSCallback)
      
    #Service Client for Lane; Lane task is the only task that will not be shutdown
    rospy.loginfo('Waiting for LaneServer to start up...')
    rospy.wait_for_service('lane_srv')
    lane_srv = rospy.ServiceProxy('lane_srv', mission_to_lane)
    rospy.loginfo('Mission Connected to LaneServer')
      
    # Action Client for PIDs
    locomotion_client = actionlib.SimpleActionClient('LocomotionServer', bbauv_msgs.msg.ControllerAction)
    locomotion_client.wait_for_server()
    rospy.loginfo("Mission connected to LocomotionServer")
      
    #Service Client for PID & Modes
    rospy.loginfo('Waiting for Set Controller Service to start up...')
    rospy.wait_for_service('set_controller_srv')
    set_ConPIDMode = rospy.ServiceProxy('set_controller_srv', set_controller)
    rospy.loginfo('Mission Connected to Set Controller Service')
      
    # Action Client for Move Base
    movebase_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    movebase_client.wait_for_server()
    rospy.loginfo("Mission connected to MovebaseServer")
    
    sm_mission = smach.StateMachine(outcomes=['mission_complete','mission_failed'])

    with sm_mission:
        smach.StateMachine.add('COUNTDOWN', Countdown(0), transitions={'succeeded':'START'})
        smach.StateMachine.add('START',Start(3,0.5,235),
                                transitions={'start_complete':'NAV_TO_GATE'})
        
        #Lane 2 Start Pos                       
        smach.StateMachine.add('NAV_TO_GATE', NavMoveBase(1,60,-9,-8.7,0.5,235), transitions={'nav_complete':'LANE_GATE_TASK', 'failed':'SURFACE'})
        #Starting right above lane1
#         smach.StateMachine.add('NAV_TO_GATE', NavMoveBase(0.5,0.5,-9,-8.7,0.5,235), transitions={'nav_complete':'LANE_GATE_TASK', 'failed':'SURFACE'})
        
        lane_gate = smach.StateMachine(outcomes=['lane_complete', 'lane_failed'])
        with lane_gate:
            smach.StateMachine.add('LANE_HOVER', HoverSearch('lane', 10, False, 1), transitions={'hover_complete':'LANE_STORE', 'failed':'LANE_SEARCH'})
            smach.StateMachine.add('LANE_SEARCH', LinearSearch('lane', 20, -1.5, 'sway', False, 1), transitions={'linear_complete':'LANE_STORE', 'failed':'LANE_SEARCH2'})
            smach.StateMachine.add('LANE_SEARCH2', LinearSearch('lane', 20, 3, 'sway', False, 1), transitions={'linear_complete':'LANE_STORE', 'failed':'lane_failed'})
            smach.StateMachine.add('LANE_STORE', StoreGlobalCoord('mission_lane_gate'), transitions={'store_complete':'LANE_GATE'})
            smach.StateMachine.add('LANE_GATE', WaitOut('lane', 60), transitions={'task_complete':'LANE_HEADINGCHANGE', 'failed':'lane_failed'})
            smach.StateMachine.add('LANE_HEADINGCHANGE', GoToHeading(10), transitions={'heading_complete':'lane_complete'})
        smach.StateMachine.add('LANE_GATE_TASK', lane_gate, transitions={'lane_complete':'TRAFFIC_TASK', 'lane_failed':'SURFACE'})        

        traffic = smach.StateMachine(outcomes=['traffic_complete', 'traffic_failed'])
        with traffic:
            smach.StateMachine.add('TRAFFIC_DEPTHCHANGE', GoToDepth(15,2), transitions={'depth_complete':'TRAFFIC_SEARCH'})
            smach.StateMachine.add('TRAFFIC_SEARCH', LinearSearch('traffic', 60, 2, 'fwd'), transitions={'linear_complete':'TRAFFIC_STORE', 'failed':'TRAFFIC_SEARCH2'})
            smach.StateMachine.add('TRAFFIC_SEARCH2', LinearSearch('traffic', 60, -1, 'sway'), transitions={'linear_complete':'TRAFFIC_STORE', 'failed':'TRAFFIC_SEARCH3'})
            smach.StateMachine.add('TRAFFIC_SEARCH3', LinearSearch('traffic', 60, 2, 'sway'), transitions={'linear_complete':'TRAFFIC_STORE', 'failed':'traffic_failed'})
            smach.StateMachine.add('TRAFFIC_STORE', StoreGlobalCoord('mission_traffic'), transitions={'store_complete':'TRAFFIC'})
            smach.StateMachine.add('TRAFFIC', WaitOut('traffic', 180), transitions={'task_complete':'TRAFFIC_DEPTHCHANGE2', 'failed':'traffic_failed'})
            smach.StateMachine.add('TRAFFIC_DEPTHCHANGE2', GoToDepth(15,0.5), transitions={'depth_complete':'traffic_complete'})
        smach.StateMachine.add('TRAFFIC_TASK', traffic, transitions={'traffic_complete':'LANE_TRAFFIC_TASK', 'traffic_failed':'SURFACE'})            

        lane_traffic = smach.StateMachine(outcomes=['lane_complete', 'lane_failed'])
        with lane_traffic:
            smach.StateMachine.add('LANE_FORWARD', GoToDistance(30, 1.5, 'fwd'), transitions={'distance_complete':'LANE_HOVER'})
            smach.StateMachine.add('LANE_HOVER', HoverSearch('lane', 10, False, 1), transitions={'hover_complete':'LANE_STORE', 'failed':'LANE_SEARCH'})
            smach.StateMachine.add('LANE_SEARCH', LinearSearch('lane', 60, 2, 'sway', False, 1), transitions={'linear_complete':'LANE_STORE', 'failed':'LANE_SEARCH2'})
            smach.StateMachine.add('LANE_SEARCH2', LinearSearch('lane', 60, -4, 'sway', False, 1), transitions={'linear_complete':'LANE_STORE', 'failed':'lane_failed'})
            smach.StateMachine.add('LANE_STORE', StoreGlobalCoord('mission_lane_traffic'), transitions={'store_complete':'LANE_TRAFFIC'})
            smach.StateMachine.add('LANE_TRAFFIC', WaitOut('lane', 60), transitions={'task_complete':'LANE_HEADINGCHANGE', 'failed':'lane_failed'})
            smach.StateMachine.add('LANE_HEADINGCHANGE', GoToHeading(10), transitions={'heading_complete':'lane_complete'})
#            smach.StateMachine.add('LANE_ABOUTURN', GoToHeading(10, 180, True), transitions={'heading_complete':'lane_complete'})      
        smach.StateMachine.add('LANE_TRAFFIC_TASK', lane_traffic, transitions={'lane_complete':'PARK_TASK', 'lane_failed':'PARK_TASK'})  

        park = smach.StateMachine(outcomes=['park_complete', 'park_failed'])
        with park:
            smach.StateMachine.add('PARK_DEPTHCHANGE', GoToDepth(10,2), transitions={'depth_complete':'PARK_SEARCH'})
            smach.StateMachine.add('PARK_SEARCH', LinearSearch('park', 60, 1, 'fwd'), transitions={'linear_complete':'PARK_STORE', 'failed':'PARK_SEARCH2'})
            smach.StateMachine.add('PARK_SEARCH2', LinearSearch('park', 60, 3, 'sway'), transitions={'linear_complete':'PARK_STORE', 'failed':'park_failed'})
            smach.StateMachine.add('PARK_STORE', StoreGlobalCoord('mission_park1'), transitions={'store_complete':'PARK'})            
            smach.StateMachine.add('PARK', WaitOut('park', 180), transitions={'task_complete':'PARK_DEPTHCHANGE2', 'failed':'park_failed'})
            smach.StateMachine.add('PARK_DEPTHCHANGE2', GoToDepth(10,0.5), transitions={'depth_complete':'park_complete'})
        smach.StateMachine.add('PARK_TASK', park, transitions={'park_complete':'LANE_PARK_TASK', 'park_failed':'SURFACE'})

        lane_park = smach.StateMachine(outcomes=['lane_complete', 'lane_failed'])
        with lane_park:
            smach.StateMachine.add('LANE_HOVER', HoverSearch('lane', 10, True, 2), transitions={'hover_complete':'LANE_STORE', 'failed':'LANE_SEARCH'})            
            smach.StateMachine.add('LANE_SEARCH', LinearSearch('lane', 40, 1, 'sway', True, 2), transitions={'linear_complete':'LANE_STORE', 'failed':'LANE_SEARCH2'})
            smach.StateMachine.add('LANE_SEARCH2', LinearSearch('lane', 40, -2, 'sway', True, 2), transitions={'linear_complete':'LANE_STORE', 'failed':'lane_failed'})            
            smach.StateMachine.add('LANE_STORE', StoreGlobalCoord('mission_lane_park1'), transitions={'store_complete':'LANE_PARK'})
            smach.StateMachine.add('LANE_PARK', WaitOut('lane', 60), transitions={'task_complete':'LANE_HEADINGCHANGE', 'failed':'lane_failed'})
            smach.StateMachine.add('LANE_HEADINGCHANGE', GoToHeading(10), transitions={'heading_complete':'lane_complete'})
        smach.StateMachine.add('LANE_PARK_TASK', lane_park, transitions={'lane_complete':'TOLL_TASK', 'lane_failed':'SURFACE'})  

        toll = smach.StateMachine(outcomes=['toll_complete', 'toll_failed'])
        with toll:
            smach.StateMachine.add('TOLL_DEPTHCHANGE', GoToDepth(3,2), transitions={'depth_complete':'TOLL_SEARCH'})
            smach.StateMachine.add('TOLL_SEARCH', LinearSearch('tollbooth', 30, 1, 'fwd'), transitions={'linear_complete':'TOLL_STORE', 'failed':'TOLL_SEARCH2'})
            smach.StateMachine.add('TOLL_SEARCH2', LinearSearch('tollbooth', 30, -2, 'sway'), transitions={'linear_complete':'TOLL_STORE', 'failed':'TOLL_SEARCH3'})
            smach.StateMachine.add('TOLL_SEARCH3', LinearSearch('tollbooth', 60, 4, 'sway'), transitions={'linear_complete':'TOLL_STORE', 'failed':'TOLL_SEARCH4'})
            smach.StateMachine.add('TOLL_SEARCH4', LinearSearch('tollbooth', 30, 1, 'fwd'), transitions={'linear_complete':'TOLL_STORE', 'failed':'TOLL_SEARCH5'})
            smach.StateMachine.add('TOLL_SEARCH5', LinearSearch('tollbooth', 60, -4, 'sway'), transitions={'linear_complete':'TOLL_STORE', 'failed':'TOLL_SEARCH6'})
            smach.StateMachine.add('TOLL_SEARCH6', LinearSearch('tollbooth', 60, 4, 'sway'), transitions={'linear_complete':'TOLL_STORE', 'failed':'toll_failed'})
            smach.StateMachine.add('TOLL_STORE', StoreGlobalCoord('toll'), transitions={'store_complete':'TOLLBOOTH'})            
            smach.StateMachine.add('TOLLBOOTH', WaitOut('tollbooth', 180), transitions={'task_complete':'toll_complete', 'failed':'toll_failed'})
        smach.StateMachine.add('TOLL_TASK', toll, transitions={'toll_complete':'SPEED_TASK', 'toll_failed':'SPEED_TASK'})

        speed = smach.StateMachine(outcomes=['speed_complete', 'speed_failed'])
        with speed:
            smach.StateMachine.add('SPEED_DEPTHCHANGE', GoToDepth(3,0.5), transitions={'depth_complete':'SPEED_SEARCH'})
#             smach.StateMachine.add('SPEED_SWAY', GoToDistance(90, 7, 'sway'), transitions={'distance_complete':'SPEED_STORE'})
            smach.StateMachine.add('SPEED_SEARCH', LinearSearch('speedtrap', 30, 1, 'sway'), transitions={'linear_complete':'SPEED_STORE', 'failed':'SPEED_SEARCH2'})
            smach.StateMachine.add('SPEED_SEARCH2', LinearSearch('speedtrap', 30, -2, 'sway'), transitions={'linear_complete':'SPEED_STORE', 'failed':'speed_failed'})
#            smach.StateMachine.add('SPEED_TO_LANE3', NavMoveBase(1,60,place='lane3'), transitions={'nav_complete':'SPEED_FWDSEARCH', 'failed':'speed_failed'})
            smach.StateMachine.add('SPEED_STORE', StoreGlobalCoord('mission_speed'), transitions={'store_complete':'SPEEDTRAP'})
            smach.StateMachine.add('SPEEDTRAP', WaitOut('speedtrap', 240), transitions={'task_complete':'speed_complete', 'failed':'speed_failed'})           
        smach.StateMachine.add('SPEED_TASK', speed, transitions={'speed_complete':'BACK_TO_LANE3', 'speed_failed':'BACK_TO_LANE3'})

        smach.StateMachine.add('BACK_TO_LANE3', NavMoveBase(3,70,place='lane3'), transitions={'nav_complete':'BACK_TO_LANE2', 'failed':'mission_failed'})        
        smach.StateMachine.add('BACK_TO_LANE2', NavMoveBase(3,70,place='lane2'), transitions={'nav_complete':'BACK_TO_LANE1', 'failed':'mission_failed'})        
        smach.StateMachine.add('BACK_TO_LANE1', NavMoveBase(3,70,place='lane1'), transitions={'nav_complete':'HOME', 'failed':'mission_failed'})
        smach.StateMachine.add('HOME', NavMoveBase(1,60,0,0,0.5,60), transitions={'nav_complete':'SURFACE', 'failed':'mission_failed'})
        smach.StateMachine.add('SURFACE', GoToDepth(3,0.12), transitions={'depth_complete':'mission_complete'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('mission_server', sm_mission, '/MISSION')
    sis.start()

    try:
        outcome = sm_mission.execute()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

########################


