#!/usr/bin/env python2

import roslib; roslib.load_manifest('mission_planner')
import rospy
from bbauv_msgs.srv import *
from bbauv_msgs.msg import *
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler, quaternion_about_axis
import dynamic_reconfigure.client
from math import pi, atan2, sqrt

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
        r = rospy.Rate(10)
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time) < self.sleep_time:
            r.sleep()
            rospy.loginfo("%d Elapsed" % (rospy.get_time() - start_time))
        return 'succeeded'        

class Start(smach.State):
    def __init__(self, timeout, start_depth, start_heading):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.name = self.__class__.__name__
        self.timeout = timeout                
        self.start_depth = start_depth
        self.start_heading = start_heading        
                
    def execute(self,userdata):
#        return 'start_complete'
        global locomotionGoal
        global locomotion_client
        global set_ConPIDMode
        global set_LocoMode
        rospy.loginfo('Executing state START')
        
        #Key in starting position here
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0, sidemove_setpoint=0,
                                            depth_setpoint= self.start_depth, 
                                            heading_setpoint = self.start_heading                                            
                                            )

        #Setting Locomotion Mode (Forward, Sidemove) ; For Default, put both to False
        try:
            resp = set_LocoMode(False, False)
            rospy.loginfo("LocoMode set to Default")
        except rospy.ServiceException, e:
            rospy.loginfo("LocoMode Fwd Default set: %s" % e)
      
        #Setting PID (Fwd? Side? Head? Depth? Pitch?) and modes (Topside? Nav?)
        try:
            resp = set_ConPIDMode(True, True, True, True, False, False, False)
            rospy.loginfo("PID and Mode is set")
        except rospy.ServiceException, e:
            rospy.loginfo("PID and Mode NOT set: %s" % e)

        #Reset DVL and Earth Odom here
#        drClient_DVL = dynamic_reconfigure.client.Client("WH_DVL")
#        drClient_DVL.update_configuration({"zero_distance":True})
        drClient_Earth = dynamic_reconfigure.client.Client("earth_odom")
        drClient_Earth.update_configuration({"zero_distance":True})        

        locomotion_client.send_goal(goal)
        locomotion_client.wait_for_result(rospy.Duration(self.timeout,0))
        rospy.loginfo("Dive Dive Dive!")
        
        locomotionGoal.depth_setpoint = self.start_depth
        locomotionGoal.heading_setpoint = self.start_heading
        locomotionGoal.sidemove_setpoint = 0
        locomotionGoal.forward_setpoint = 0
        return 'succeeded'

class GoToDistance(smach.State):
    def __init__(self, timeout, distance, direction):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.distance = distance
        self.timeout = timeout
        self.direction = direction
        
    def execute(self,userdata):
        global locomotionGoal
        global locomotion_client
        global set_LocoMode
        
        if self.direction == 'fwd':

            #Setting Locomotion Mode (Forward, Sidemove) ; For Default, put both to False
            try:
                resp = set_LocoMode(True, False)
                rospy.loginfo("LocoMode set to Fwd")
            except rospy.ServiceException, e:
                rospy.loginfo("LocoMode Fwd NOT set: %s" % e)
            
            goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=self.distance,
                                                sidemove_setpoint=0,
                                                depth_setpoint=locomotionGoal.depth_setpoint,
                                                heading_setpoint=locomotionGoal.heading_setpoint)
        
        if self.direction == 'sway':
            
            #Setting Locomotion Mode (Forward, Sidemove) ; For Default, put both to False
            try:
                resp = set_LocoMode(False, True)
                rospy.loginfo("LocoMode set to Sway")
            except rospy.ServiceException, e:
                rospy.loginfo("LocoMode Sway NOT set: %s" % e)
                
            goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0,
                                                sidemove_setpoint=self.distance,
                                                depth_setpoint=locomotionGoal.depth_setpoint,
                                                heading_setpoint=locomotionGoal.heading_setpoint)

        rospy.loginfo('Going %s distance %s' % (str(self.direction), str(self.distance)))                                            
        locomotion_client.send_goal(goal)
        locomotion_client.wait_for_result(rospy.Duration(self.timeout,0))

        locomotionGoal.depth_setpoint = goal.depth_setpoint 
        locomotionGoal.heading_setpoint = goal.heading_setpoint
             
        return 'succeeded'

class GoToDepth(smach.State):
    def __init__(self, timeout=3, depth=None, surface=False):
        smach.State.__init__(self, outcomes=['succeeded'])
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
        return 'succeeded'

class GoToHeading(smach.State):
    def __init__(self, timeout=3, heading=None, relative=False):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.heading = heading
        self.relative = relative
        self.timeout = timeout

    def normalize_angle(self, angle):
        normalized = (angle%360+360)%360
        return normalized
        
    def execute(self,userdata):
        global locomotionGoal
        global locomotion_client
        global set_LocoMode
        
        #Setting Locomotion Mode (Forward, Sidemove) ; For Default, put both to False
        try:
            resp = set_LocoMode(False, False)
            rospy.loginfo("LocoMode set to Default")
        except rospy.ServiceException, e:
            rospy.loginfo("LocoMode Default NOT set: %s" % e)        
        
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
        return 'succeeded'

    
class StoreGlobalCoord(smach.State):
    def __init__(self, task_name):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.name = self.__class__.__name__ 
        self.task_name = task_name
            
    def execute(self, userdata):
        global global_x
        global global_y
        global global_depth
        global global_heading
        global marker_id
        
#       Publishing marker so that RVIZ can visualize
#         task_marker_pub = rospy.Publisher("/task_visualization", Marker)
#         task_marker = Marker()
#         
#         task_marker.header.frame_id = "/odom"
#         task_marker.header.stamp = rospy.Time.now()
#         task_marker.id = marker_id        
#         #increment marker_id because each marker must have unique id
#         marker_id += 1
#         task_marker.type = task_marker.CUBE
#         task_marker.action = task_marker.ADD
#         task_marker.scale.x = 1
#         task_marker.scale.y = 1
#         task_marker.scale.z = 1
#         task_marker.color.a = 1
#         task_marker.color.r = 1
#         task_marker.color.g = 1
#         task_marker.color.b = 0
#         task_marker.pose.position.x = global_x
#         task_marker.pose.position.y = global_y
#         task_marker.pose.position.z = global_altitude
#         task_marker.pose.orientation.x = 0
#         task_marker.pose.orientation.y = 0
#         task_marker.pose.orientation.z = 0
#         task_marker.pose.orientation.w = 1  
#         task_marker.text = self.task_name
#         task_marker_pub.publish(task_marker) #Needs to be published constantly
       
        rospy.loginfo('x=%s y=%s depth=%s heading=%s' % (str(global_x), str(global_y), str(global_depth), str(global_heading)))
        rospy.set_param(self.task_name+'/x', global_x)
        rospy.set_param(self.task_name+'/y', global_y)
        rospy.set_param(self.task_name+'/depth', global_depth)
        rospy.set_param(self.task_name+'/heading', global_heading)
        rospy.set_param(self.task_name+'/altitude', global_altitude)
                    
        return 'succeeded'
        
class HoverSearch(smach.State):
    def __init__(self, task_name, timeout, use_left=False, num_lanes=0, start_depth=None, start_heading=None):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.name = self.__class__.__name__
        self.task_name = task_name
        self.task_srv_name = task_name + '_srv'
        self.task_srv = None
        self.timeout = timeout
        self.use_left = use_left
        self.num_lanes = num_lanes
        self.start_depth = start_depth
        self.start_heading = start_heading
        
    def execute(self, userdata):
        global locomotionGoal
        global locomotion_client
        global set_ConPIDMode
        global set_LocoMode
        global lane_srv
        global isSearchDone
        global isSearchFailed
        isSearchFailed = False
        isSearchDone = False

        rospy.loginfo("Entering %s %s state" % (self.task_name, self.name))

        #Setting Locomotion Mode (Forward, Sidemove) ; For Default, put both to False
        try:
            resp = set_LocoMode(False, False)
            rospy.loginfo("LocoMode set to Default")
        except rospy.ServiceException, e:
            rospy.loginfo("LocoMode Default NOT set: %s" % e)
        
        #Setting PID (Fwd? Side? Head? Depth? Pitch?) and modes (Topside? Nav?)
        try:
            resp = set_ConPIDMode(True, True, True, True, False, False, False)
            rospy.loginfo("PID and Mode is set")
        except rospy.ServiceException, e:
            rospy.loginfo("PID and Mode NOT set: %s" % e)
            return 'failed'        

        #connecting to task server
        if self.task_name != 'lane':
            rospy.loginfo('Mission attempt connecting to %s Server' % self.task_name)           
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

        #Begin Searching For Task
        if self.task_name == 'lane':
            try:
                resp = lane_srv(True,locomotionGoal,self.use_left,self.num_lanes,False)
            except rospy.ServiceException, e:
                rospy.loginfo("Failed to start Search: %s" % e)
                return 'failed'  
        if self.task_name != 'lane':
            try:
                resp = self.task_srv(True, locomotionGoal, False)
                rospy.loginfo("Searching for %s" % self.task_name)
            except rospy.ServiceException, e:
                rospy.loginfo("Failed to start Search: %s" % e)
                return 'failed'

        r = rospy.Rate(30)
        start_time = rospy.get_time()
        rospy.loginfo("Hovering to search for %s" % self.task_name)
        while (not rospy.is_shutdown()) and ((rospy.get_time()-start_time) <= self.timeout):
            if not isSearchDone:
                #Hovering
                locomotion_client.send_goal(goal)
                locomotion_client.wait_for_result(rospy.Duration(1,0))
                                    
            if isSearchDone:
                rospy.loginfo("Found %s. %d of %d secs elapsed" % (self.task_name, rospy.get_time()-start_time, self.timeout))
                return 'succeeded'
            
            if isSearchFailed:
                rospy.loginfo("Failed to find %s. %d of %d secs elapsed" % (self.task_name, rospy.get_time()-start_time, self.timeout))
                return 'failed'
            
            r.sleep()
        
        #Timed out
        if self.task_name == 'lane':
            try:
                rospy.loginfo('Timed Out: Failed to find %s' % self.task_name)
                resp = lane_srv(False, locomotionGoal, False, 1, True)            
                return 'failed'              
            except rospy.ServiceException, e:
                rospy.loginfo("Timed Out: Failed to abort: %s" % e)    
                return 'failed'                         
        if self.task_name != 'lane':
            try:
                rospy.loginfo('Timed Out: Failed to find %s' % self.task_name)
                resp = self.task_srv(False, locomotionGoal, True)            
                return 'failed'              
            except rospy.ServiceException, e:
                rospy.loginfo("Timed Out: Failed to abort: %s" % e)
                return 'failed'                   
                
class LinearSearch(smach.State):
    def __init__(self, task_name, timeout, distance, direction, use_left=False, num_lanes=0, start_depth=None, start_heading=None):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
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
            rospy.loginfo("Search Motion Callback Complete")
            self.motionStatus = True
                   
    def execute(self, userdata):
        global locomotionGoal
        global locomotion_client
        global set_ConPIDMode
        global set_LocoMode
        global lane_srv
        global isSearchDone
        global isSearchFailed
        isSearchFailed = False
        isSearchDone = False
        
        rospy.loginfo("Entering %s %s state" % (self.task_name, self.name))  
        
        #Setting Locomotion Mode (Forward, Sidemove) ; For Default, put both to False
        try:
            resp = set_LocoMode(False, False)
            rospy.loginfo("LocoMode set to Default")
        except rospy.ServiceException, e:
            rospy.loginfo("LocoMode Default NOT set: %s" % e)

        #Setting PID (Fwd? Side? Head? Depth? Pitch?) and modes (Topside? Nav?)
        try:
            resp = set_ConPIDMode(True, True, True, True, False, False, False)
            rospy.loginfo("PID and Mode is set")
        except rospy.ServiceException, e:
            rospy.loginfo("PID and Mode NOT set: %s" % e)
            return 'failed'
        
        #connecting to task server 

        if self.task_name != 'lane':
            rospy.loginfo('Mission attempt connecting to %s Server' % self.task_name)           
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
                rospy.loginfo("Failed to start Search: %s" % e)
                return 'failed'  
        if self.task_name != 'lane':
            try:
                resp = self.task_srv(True, locomotionGoal, False)
                rospy.loginfo("Searching for %s" % self.task_name)
            except rospy.ServiceException, e:
                rospy.loginfo("Failed to start Search: %s" % e)
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
        rospy.loginfo("Moving %s %s meters to search for %s" % (self.direction, str(self.distance) ,self.task_name))
        while (not rospy.is_shutdown()) and ((rospy.get_time()-start_time) <= self.timeout):
            
            if not isSearchDone:
                if self.motionStatus:
                    return 'failed'
                                    
            if isSearchDone:
                locomotionGoal.depth_setpoint = goal.depth_setpoint 
                locomotionGoal.heading_setpoint = goal.heading_setpoint
                locomotion_client.cancel_all_goals()             
                rospy.loginfo("Found %s. %d of %d secs elapsed" % (self.task_name, rospy.get_time()-start_time, self.timeout))
                return 'succeeded'
            
            if isSearchFailed:
                locomotionGoal.depth_setpoint = goal.depth_setpoint 
                locomotionGoal.heading_setpoint = goal.heading_setpoint
                locomotion_client.cancel_all_goals()
                rospy.loginfo("Failed to find %s. %d of %d secs elapsed" % (self.task_name, rospy.get_time()-start_time, self.timeout))
                return 'failed'            
            r.sleep()
        
        #Timed out
        if self.task_name == 'lane':
            try:
                rospy.loginfo('Timed Out: Failed to find %s' % self.task_name)
                resp = lane_srv(False, locomotionGoal, False, 1, True)            
                return 'failed'              
            except rospy.ServiceException, e:
                rospy.loginfo("Timed Out: Failed to abort: %s" % e)    
                return 'failed'                         
        if self.task_name != 'lane':
            try:
                rospy.loginfo('Timed Out: Failed to find %s' % self.task_name)
                resp = self.task_srv(False, locomotionGoal, True)            
                return 'failed'              
            except rospy.ServiceException, e:
                rospy.loginfo("Timed Out: Failed to abort: %s" % e)
                return 'failed'                 
        
class WaitOut(smach.State):
    def __init__(self, task_name, timeout):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.name = self.__class__.__name__ 
        self.task_name = task_name
        self.task_srv_name = task_name + '_srv'
        self.task_srv = None
        self.timeout = timeout    
        
    def execute(self, userdata):
        global lane_srv
        global locomotion_client
        global locomotionGoal
        global isTaskComplete
        global isTaskFailed
        isTaskFailed = False
        isTaskComplete = False     
           
        rospy.loginfo("Entering %s %s state" % (self.task_name, self.name))  

        #connecting to task server;      
        if self.task_name != 'lane':        
            #connecting to task server
            rospy.wait_for_service(self.task_srv_name)   
            self.task_srv = rospy.ServiceProxy(self.task_srv_name, mission_to_vision)
            rospy.loginfo('Mission Connected to %s Server' % self.task_name)        
        
        #Waiting Out
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        rospy.loginfo("%s: Found. Task Controlling Vehicle" % (self.task_name))
        while (not rospy.is_shutdown()) and ((rospy.get_time()-start_time) <= self.timeout):
            if isTaskComplete:               
                rospy.loginfo("Completed %s. %d of %d secs elapsed" % (self.task_name, rospy.get_time()-start_time, self.timeout))
                return 'succeeded'
            if isTaskFailed:
              
                rospy.loginfo("Failed to complete %s. %d of %d secs elapsed" % (self.task_name, rospy.get_time()-start_time, self.timeout))
                return 'failed'
            r.sleep()
        
        #Aborting task due to timeout
        if self.task_name == 'lane':
            try:
                rospy.loginfo('Timed Out: Failed to complete %s' % self.task_name)
                resp = lane_srv(False, locomotionGoal, False, 1, True)            
                return 'failed'              
            except rospy.ServiceException, e:
                rospy.loginfo("Timed Out: Failed to abort: %s" % e)
                return 'failed'                              
     
        if self.task_name != 'lane':
            try:
                rospy.loginfo('Timed Out: Failed to complete %s' % self.task_name)
                resp = self.task_srv(False, locomotionGoal, True)            
                return 'failed'              
            except rospy.ServiceException, e:
                rospy.loginfo("Timed Out: Failed to abort: %s" % e)            
                return 'failed'

class WaitOutAndSearch(smach.State):
    def __init__(self, waitout_task_name, search_task_name, timeout):
        smach.State.__init__(self, outcomes=['task_succeeded','search_succeeded', 'failed'])
        self.name = self.__class__.__name__ 

        self.waitout_task_name = waitout_task_name
        self.waitout_task_srv_name = waitout_task_name + '_srv'
        self.waitout_task_srv = None

        self.search_task_name = search_task_name
        self.search_task_srv_name = search_task_name + '_srv'
        self.search_task_srv = None

        self.timeout = timeout    
        
    def execute(self, userdata):
        global lane_srv
        global locomotion_client
        global locomotionGoal
        
        global isSearchDone
        global isSearchFailed
        global isTaskComplete
        global isTaskFailed
        isTaskFailed = False
        isTaskComplete = False     
        isSearchDone = False
        isSearchFailed = False
        
        rospy.loginfo("Entering %s %s state; Searching for %s at the same time" % (self.waitout_task_name, self.name, self.search_task_name))  
        
        print 'testing sending non positive depth to Eng Wei'
        locomotionGoal.depth_heading = 0.5
        
        #Connecting to Waiout and Search task server;
        if self.waitout_task_name != 'lane':
            rospy.wait_for_service(self.waitout_task_srv_name)
            self.waitout_task_srv = rospy.ServiceProxy(self.waitout_task_srv_name, mission_to_vision)
            rospy.loginfo('Mission Connected to %s Server' % self.waitout_task_name)

        if self.search_task_name != 'lane':
            rospy.wait_for_service(self.search_task_srv_name)   
            self.search_task_srv = rospy.ServiceProxy(self.search_task_srv_name, mission_to_vision)
            rospy.loginfo('Mission Connected to %s Server' % self.search_task_name)      

        #Begin Searching For Task
        if self.search_task_name == 'lane':
            try:
                resp = lane_srv(True,locomotionGoal,self.use_left,self.num_lanes,False)
            except rospy.ServiceException, e:
                rospy.loginfo("Failed to start Search" % e)
                return 'failed'  
        if self.search_task_name != 'lane':
            try:
                resp = self.search_task_srv(True, locomotionGoal, False)
                rospy.loginfo("Searching for %s" % self.search_task_name)
            except rospy.ServiceException, e:
                rospy.loginfo("Failed to start Search" % e)
                return 'failed'
        
        #Waiting Out
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        rospy.loginfo("%s: Found. Task Controlling Vehicle" % (self.waitout_task_name))
        while (not rospy.is_shutdown()) and ((rospy.get_time()-start_time) <= self.timeout):

            if isSearchDone:
                rospy.loginfo("Found %s. %d of %d secs elapsed" % (self.search_task_name, rospy.get_time()-start_time, self.timeout))
                
                #Aborting WaitOut task due to search task found
                if self.waitout_task_name == 'lane':
                    try:
                        rospy.loginfo('Found %s during Waitout, Aborting %s' % (self.search_task_name, self.waitout_task_name))
                        resp = lane_srv(False, locomotionGoal, False, 1, True)
                    except rospy.ServiceException, e:
                        rospy.loginfo("Timed Out: Failed to abort: %s" % e)
                        return 'failed'                              
             
                if self.waitout_task_name != 'lane':
                    try:
                        rospy.loginfo('Found %s during Waitout, Aborting %s' % (self.search_task_name, self.waitout_task_name))
                        resp = self.waitout_task_srv(False, locomotionGoal, True)            
                    except rospy.ServiceException, e:
                        rospy.loginfo("Failed to abort: %s" % e)            
                        return 'failed'
                return 'search_succeeded'

            if isTaskComplete:
                rospy.loginfo("Task Completed %s. %d of %d secs elapsed" % (self.waitout_task_name, rospy.get_time()-start_time, self.timeout))


                #Aborting Search task due to waitout task complete
                if self.search_task_name == 'lane':
                    try:
                        rospy.loginfo('Task Complete, Failed to find %s' % self.search_task_name)
                        resp = lane_srv(False, locomotionGoal, False, 1, True)            
                    except rospy.ServiceException, e:
                        rospy.loginfo("Timed Out: Failed to abort: %s" % e)
                        return 'failed'                              
             
                if self.search_task_name != 'lane':
                    try:
                        rospy.loginfo('Task Complete, Failed to find %s' % self.search_task_name)
                        resp = self.search_task_srv(False, locomotionGoal, True)            
                    except rospy.ServiceException, e:
                        rospy.loginfo("Failed to abort: %s" % e)            
                        return 'failed'

                return 'task_succeeded'

            #As of 18th July2013, only the Acoustics task tell mission that Search has failed. 
            #All vision task will keep searching and have no fail condition
            if isSearchFailed:
                
                rospy.loginfo("Search Failed %s. %d of %d secs elapsed" % (self.search_task_name, rospy.get_time()-start_time, self.timeout))
                return 'failed'

            if isTaskFailed:
                rospy.loginfo("Task Failed %s. %d of %d secs elapsed" % (self.waitout_task_name, rospy.get_time()-start_time, self.timeout))

                #Aborting Search task due to waitout task failed
                if self.search_task_name == 'lane':
                    try:
                        rospy.loginfo('Task Failed, Failed to find %s' % self.search_task_name)
                        resp = lane_srv(False, locomotionGoal, False, 1, True)            
                    except rospy.ServiceException, e:
                        rospy.loginfo("Timed Out: Failed to abort: %s" % e)
                        return 'failed'                              
             
                if self.search_task_name != 'lane':
                    try:
                        rospy.loginfo('Task Failed, Failed to find %s' % self.search_task_name)
                        resp = self.search_task_srv(False, locomotionGoal, True)            
                    except rospy.ServiceException, e:
                        rospy.loginfo("Failed to abort: %s" % e)            
                        return 'failed'

                return 'failed'
            r.sleep()
        
        #Aborting task due to timeout
              
        if self.waitout_task_name == 'lane':
            try:
                rospy.loginfo('Timed Out: Failed to complete %s' % self.waitout_task_name)
                resp = lane_srv(False, locomotionGoal, False, 1, True)            
                return 'failed'              
            except rospy.ServiceException, e:
                rospy.loginfo("Timed Out: Failed to abort: %s" % e)
                return 'failed'                              
     
        if self.waitout_task_name != 'lane':
            try:
                rospy.loginfo('Timed Out: Failed to complete %s' % self.waitout_task_name)
                resp = self.waitout_task_srv(False, locomotionGoal, True)            
                return 'failed'              
            except rospy.ServiceException, e:
                rospy.loginfo("Timed Out: Failed to abort: %s" % e)            
                return 'failed'

        if self.search_task_name == 'lane':
            try:
                rospy.loginfo('Timed Out: Failed to find %s' % self.search_task_name)
                resp = lane_srv(False, locomotionGoal, False, 1, True)            
                return 'failed'              
            except rospy.ServiceException, e:
                rospy.loginfo("Timed Out: Failed to abort: %s" % e)
                return 'failed'                              
     
        if self.search_task_name != 'lane':
            try:
                rospy.loginfo('Timed Out: Failed to find %s' % self.search_task_name)
                resp = self.search_task_srv(False, locomotionGoal, True)            
                return 'failed'              
            except rospy.ServiceException, e:
                rospy.loginfo("Timed Out: Failed to abort: %s" % e)            
                return 'failed'

class Nav(smach.State):
    
    def __init__ (self, prep_timeout, nav_timeout, end_timeout, x=0, y=0, start_depth = 0.5, end_depth=0.5, end_heading=0, place= None): #yaw here is BBAUV's controller convention'
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.name = self.__class__.__name__
        self.x = x
        self.y = y
        
        self.distanceToCoord = 0
        self.start_depth = start_depth        
        self.start_heading = 0
        self.end_depth = end_depth
        self.end_heading = end_heading
        
        self.place = place
        self.prep_timeout = prep_timeout    
        self.nav_timeout = nav_timeout  
        self.end_timeout = end_timeout

    def start_heading_gen(self, x,y,global_x,global_y):
    
        y_diff = abs(y - global_y)
        x_diff = abs(x - global_x)
        
        #Checking if both are the same
        if y_diff == 0 and x_diff == 0:
            print 'Destination and current location is the same!'
            start_heading = 0
            return start_heading
        
        #Checking for case when moving along 1 of 4 cardinal points
        if y_diff ==0:
            if x - global_x < 0: start_heading = 0
            if x - global_x > 0: start_heading = 180
            return start_heading
        if x_diff == 0:
            if y - global_y > 0: start_heading = 90
            if y - global_y < 0: start_heading = 270
            return start_heading
        
        if (x - global_x)>0 and (y - global_y)>0:
            start_heading = 90 - atan2(x_diff,y_diff)*(180/pi)
    
        if (x - global_x)<0 and (y - global_y)>0:
            start_heading = 90 + atan2(x_diff,y_diff)*(180/pi)
                        
        if (x - global_x)<0 and (y - global_y)<0:
            start_heading = 270 - atan2(x_diff,y_diff)*(180/pi)
                        
        if (x - global_x)>0 and (y - global_y)<0:
            start_heading = 270 + atan2(x_diff,y_diff)*(180/pi)
                                        
        return start_heading

    def distanceToGlobalCoord(self, x,y,global_x,global_y):
        
        y_diff = abs(y - global_y)
        x_diff = abs(x - global_x)
        
        distance = sqrt(pow(y_diff,2)+pow(x_diff,2))
        
        return distance
    
    def normalize_angle(self, angle):
        normalized = (angle%360+360)%360
        return normalized
        
    def execute (self, userdata):
        
        global locomotion_client
        global locomotionGoal
        global set_ConPIDMode
        
        move_base_mode = False
        simple_nav_mode = True
        
        #Check is need to get cooridnate from param server; if place is not None but a string, use string to query param server for nav coords; if place is None, use x,y and yaw            
        if self.place != None:        
            self.x = rospy.get_param(self.place + '/x')
            self.y = rospy.get_param(self.place + '/y')
            self.end_depth = rospy.get_param(self.place + '/depth')
            self.end_heading = rospy.get_param(self.place + '/heading')


        if simple_nav_mode:

            #Setting Locomotion Mode (Forward, Sidemove) ; For Default, put both to False
            try:
                resp = set_LocoMode(False, False)
                rospy.loginfo("LocoMode set to Default by Navigation State")
            except rospy.ServiceException, e:
                rospy.loginfo("LocoMode Default NOT set: %s" % e) 
    
            #Setting PID (Fwd? Side? Head? Depth? Pitch?) and modes (Topside? Nav?)
            try:
                resp = set_ConPIDMode(True, True, True, True, True, False, False)
                rospy.loginfo("PID and Mode is set")
            except rospy.ServiceException, e:
                rospy.loginfo("PID and Mode NOT set: %s" % e)
            
            rospy.loginfo("Generated Start heading for nav")    
            self.start_heading = self.start_heading_gen(self.x, self.y, global_x, global_y)
                
            #Change Depth and Face Heading First
            goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0,sidemove_setpoint=0,depth_setpoint=self.start_depth,heading_setpoint=self.start_heading)
    
            locomotion_client.send_goal(goal) 
            rospy.loginfo('Starting Navigation from depth %s m and facing yaw=%s deg' % (str(self.start_depth), str(self.start_heading)))
            locomotion_client.wait_for_result(rospy.Duration(self.prep_timeout,0))

            self.start_heading = self.start_heading_gen(self.x, self.y, global_x, global_y)
            self.distanceToCoord = self.distanceToGlobalCoord(self.x, self.y, global_x, global_y)

            #Navigating to global coord
            goal2 = bbauv_msgs.msg.ControllerGoal(forward_setpoint=self.distanceToCoord,sidemove_setpoint=0,depth_setpoint=self.start_depth,heading_setpoint=self.start_heading)
    
            locomotion_client.send_goal(goal2)
            rospy.loginfo('Navigating to x=%s y=%s.' % (str(self.x), str(self.y)))
            locomotion_client.wait_for_result(rospy.Duration(self.nav_timeout,0))

            self.start_heading = self.start_heading_gen(self.x, self.y, global_x, global_y)
            self.distanceToCoord = self.distanceToGlobalCoord(self.x, self.y, global_x, global_y)

            #Navigating to global coord
            goal3 = bbauv_msgs.msg.ControllerGoal(forward_setpoint=self.distanceToCoord,sidemove_setpoint=0,depth_setpoint=self.start_depth,heading_setpoint=self.start_heading)
    
            locomotion_client.send_goal(goal3)
            rospy.loginfo('Navigating to x=%s y=%s.' % (str(self.x), str(self.y)))
            locomotion_client.wait_for_result(rospy.Duration(self.nav_timeout,0))
            
        if move_base_mode:
            global movebase_client
            movebaseGoal = MoveBaseGoal()
            
        #Generate Start_Heading to point vehicle in the direction of travel
        
            self.start_heading = self.start_heading_gen(self.x, self.y, global_x, global_y)
                
            #Change Depth and Face Heading First
            goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0,sidemove_setpoint=0,depth_setpoint=self.start_depth,heading_setpoint=self.start_heading)
    
            locomotion_client.send_goal(goal)
            rospy.loginfo('Starting Navigation from depth %s m and facing yaw=%s deg' % (str(self.start_depth), str(self.start_heading)))
            locomotion_client.wait_for_result(rospy.Duration(self.prep_timeout,0))
                    
            #convert yaw to move_base convention
            ros_heading = self.normalize_angle((360-(self.start_heading))) * (pi/180) #self.start_heading * (pi/180) 
            x,y,z,w = quaternion_from_euler(0,0,ros_heading) #input must be radians
            rospy.logdebug('z= %s w= %s' % (str(z),str(w)))
    
            #Setting Locomotion Mode (Forward, Sidemove) ; For Default, put both to False
            try:
                resp = set_LocoMode(False, False)
                rospy.loginfo("LocoMode set to Default")
            except rospy.ServiceException, e:
                rospy.loginfo("LocoMode Default NOT set: %s" % e) 
    
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
            rospy.loginfo('Navigating t_geno x=%s y=%s. Facing yaw=%s' % (str(self.x), str(self.y), str(self.start_heading)))
            movebase_client.wait_for_result(rospy.Duration(self.nav_timeout,0))
    
            #Setting PID (Fwd? Side? Head? Depth? Pitch?) and modes (Topside? Nav?)
            try:
                resp = set_ConPIDMode(True, True, True, True, False, False, False)
                rospy.loginfo("PID and Mode is set")
            except rospy.ServiceException, e:
                rospy.loginfo("PID and Mode NOT set: %s" % e)
        
        #End Navigation with the following heading and depth    
        goal.depth_setpoint = self.end_depth
        goal.heading_setpoint = self.end_heading
        locomotion_client.send_goal(goal)
        rospy.loginfo('Ending navigation at depth %s and facing yaw=%s' % (str(self.end_depth), str(self.end_heading)))
        locomotion_client.wait_for_result(rospy.Duration(self.end_timeout,0))
        rospy.loginfo('Reached depth %s. Facing yaw=%s' % (str(self.end_depth), str(self.end_heading)))
        
        #Update locomotionGoal
        locomotionGoal.depth_setpoint = self.end_depth
        locomotionGoal.heading_setpoint = self.end_heading
        return 'succeeded'
     
def handle_srv(req):
    global locomotionGoal
    global isSearchDone
    global isSearchFailed
    global isTaskComplete
    global isTaskFailed
    
    search_call = False
    task_call = False
    
    rospy.loginfo("Mission Server Received A Service Call")

    #Search completion request from Vision Node.
    if(req.search_request):
        isSearchDone = True
        search_call = True
        rospy.logdebug("Search complete")
    
    if not (req.search_request):
        isSearchFailed = True
        search_call = True
        rospy.logdebug("Search Failed To Complete")

    #Task completion request from Vision Node.
    if(req.task_complete_request):
        isTaskComplete = True
        task_call = True
        rospy.logdebug("Task complete")
        locomotionGoal = req.task_complete_ctrl 

    if not (req.task_complete_request):
        isTaskFailed = True
        task_call = True
        rospy.logdebug("Task Failed To Complete")
        locomotionGoal = req.task_complete_ctrl 
    
    if search_call:
        return vision_to_missionResponse(True,False) 
    if task_call:
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

def AltitudeCallback(msg):
    global global_altitude
    global_altitude = msg.data

locomotion_client = None
locomotionGoal = controller()
movebase_client = None
set_ConPIDMode = None
set_LocoMode = None

mission_server = None
isSearchDone = False
isSearchFailed = False
isTaskComplete = False
isTaskFailed = False
lane_srv = None

global_x = 0
global_y = 0
global_heading = 0
global_depth = 0.3
global_altitude = 0
marker_id = 1

if __name__ == '__main__':
    rospy.init_node('Mission_planner', anonymous=True)
    
    test_mode = False
    
    mission_server = rospy.Service('mission_srv', vision_to_mission, handle_srv)
    rospy.loginfo('MissionServer Initialized!')

    if not test_mode:    
        # Subscribing to Telemetry
        odom_sub = rospy.Subscriber('/earth_odom', Odometry, odomCallback)
        depth_sub = rospy.Subscriber('/depth', depth, depthCallback)
        AHRS_sub = rospy.Subscriber('/AHRS8_data_e', imu_data, AHRSCallback)
        Altitude_sub = rospy.Subscriber('/altitude', Float32, AltitudeCallback)
          
        #Service Client for Lane; Lane task is the only task that will not be shutdown
        rospy.loginfo('Mission Waiting for LaneServer to start up...')
        rospy.wait_for_service('lane_srv')
        lane_srv = rospy.ServiceProxy('lane_srv', mission_to_lane)
        rospy.loginfo('Mission Connected to LaneServer')
          
        # Action Client for PIDs
        locomotion_client = actionlib.SimpleActionClient('LocomotionServer', bbauv_msgs.msg.ControllerAction)
        rospy.loginfo('Mission Waiting for Locomotion Server to start up...')
        locomotion_client.wait_for_server()
        rospy.loginfo("Mission connected to LocomotionServer")
          
        #Service Client for PID & Modes
        rospy.loginfo('Mission Waiting for Set Controller Service to start up...')
        rospy.wait_for_service('set_controller_srv')
        set_ConPIDMode = rospy.ServiceProxy('set_controller_srv', set_controller)
        rospy.loginfo('Mission Connected to Set Controller Service')
        
        rospy.loginfo('Mission Waiting for Locomotion Modes Service to start up...')
        rospy.wait_for_service('locomotion_mode_srv')
        set_LocoMode = rospy.ServiceProxy('locomotion_mode_srv',locomotion_mode)
        rospy.loginfo('Mission Connected to Locomotion Mode Service')
        
        # Action Client for Move Base
        movebase_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo('Mission Waiting for Move Base Service to start up...')
        movebase_client.wait_for_server()
        rospy.loginfo("Mission connected to MovebaseServer")

### Insert Mission Here ###
 
    sm_mission = smach.StateMachine(outcomes=['mission_complete','mission_failed'])

    with sm_mission:
        smach.StateMachine.add('COUNTDOWN', Countdown(0), transitions={'succeeded':'START'})
        smach.StateMachine.add('START',Start(10,0.5,70),transitions={'succeeded':'HOVER'})
        smach.StateMachine.add('HOVER', HoverSearch('acoustic', 30, start_depth=0.5, start_heading=70), transitions={'succeeded':'TASK', 'failed':'GOFWD'})

        smach.StateMachine.add('GOFWD', GoToDistance(40, 9, 'fwd'), transitions={'succeeded':'HOVER2'})
        smach.StateMachine.add('HOVER2', HoverSearch('acoustic', 30, start_depth=0.5, start_heading=70), transitions={'succeeded':'TASK', 'failed':'GOFWD2'})

        smach.StateMachine.add('GOFWD2', GoToDistance(40, 2, 'fwd'), transitions={'succeeded':'HOVER3'})
        smach.StateMachine.add('HOVER3', HoverSearch('acoustic', 30, start_depth=0.5, start_heading=70), transitions={'succeeded':'TASK', 'failed':'GOFWD3'})

        smach.StateMachine.add('GOFWD3', GoToDistance(40, 2, 'fwd'), transitions={'succeeded':'HOVER4'})
        smach.StateMachine.add('HOVER4', HoverSearch('acoustic', 30, start_depth=0.5, start_heading=70), transitions={'succeeded':'TASK', 'failed':'SURFACE_SADLY'})

        smach.StateMachine.add('TASK', WaitOutAndSearch('acoustic','drivethru', 180), transitions={'task_succeeded':'HOVER5', 'search_succeeded':'TASK2','failed':'SURFACE_SADLY'})

        smach.StateMachine.add('TASK2', WaitOut('drivethru', 180), transitions={'succeeded':'SURFACE_VICTORIOUSLY', 'failed':'SURFACE_SADLY'})

        smach.StateMachine.add('HOVER5', HoverSearch('drivethru', 120), transitions={'succeeded':'TASK2', 'failed':'SEARCH_LEFT'})    
        smach.StateMachine.add('SEARCH_LEFT', LinearSearch('drivethru', 30, -1, 'sway'), transitions={'succeeded':'TASK2', 'failed':'SEARCH_RIGHT'})
        smach.StateMachine.add('SEARCH_RIGHT', LinearSearch('drivethru', 30, 2, 'sway'), transitions={'succeeded':'TASK2', 'failed':'SURFACE_SADLY'})
       
        smach.StateMachine.add('SURFACE_SADLY', GoToDepth(20, 0.5), transitions={'succeeded':'mission_failed'})
        smach.StateMachine.add('SURFACE_VICTORIOUSLY', GoToDepth(5, 0.5), transitions={'succeeded':'VICTORY_SPIN'})
        smach.StateMachine.add('VICTORY_SPIN', GoToHeading(60, 0, relative=True), transitions={'succeeded':'mission_complete'})
 
#    sm_mission = smach.StateMachine(outcomes=['mission_complete','mission_failed'])

#    with sm_mission:
#        smach.StateMachine.add('COUNTDOWN', Countdown(0), transitions={'succeeded':'START'})
#        smach.StateMachine.add('START',Start(10,0.26,70),transitions={'succeeded':'GO_FWD'})
#        smach.StateMachine.add('GO_FWD', GoToDistance(80, 11, 'fwd'), transitions={'succeeded':'HOVER'})
#        smach.StateMachine.add('HOVER', HoverSearch('tollbooth', 30), transitions={'succeeded':'TOLLBOOTH', 'failed':'MOVERIGHT'})
#        smach.StateMachine.add('TOLLBOOTH', WaitOut('tollbooth', 180), transitions={'succeeded':'RISE', 'failed': 'RISE'})
#        smach.StateMachine.add('RISE', GoToDepth(10, 0.26), transitions={'succeeded':'FACEWALL'})
#        smach.StateMachine.add('FACEWALL', GoToHeading(10, 90), transitions={'succeeded':'MOVERIGHT'})
#        smach.StateMachine.add('MOVERIGHT', LinearSearch('speedtrap', 60, 4, 'sway'), transitions={'succeeded':'TASK', 'failed':'SURFACE_SADLY'})
#        smach.StateMachine.add('TASK', WaitOut('speedtrap', 120), transitions={'succeeded':'SURFACE_VICTORIOUSLY', 'failed':'SURFACE_SADLY'})
#        smach.StateMachine.add('SURFACE_SADLY', GoToDepth(20, 0.26), transitions={'succeeded':'mission_failed'})
#        smach.StateMachine.add('SURFACE_VICTORIOUSLY', GoToDepth(5, 0.26), transitions={'succeeded':'VICTORY_SPIN'})
#        smach.StateMachine.add('VICTORY_SPIN', GoToHeading(60, 0, relative=True), transitions={'succeeded':'mission_complete'})

### Mission Ends Here ###  

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('mission_server', sm_mission, '/MISSION')
    sis.start()

    try:
        outcome = sm_mission.execute()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

########################



