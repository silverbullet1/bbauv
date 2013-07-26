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

#import subprocess

import actionlib
import smach
import smach_ros
from smach import Sequence
from smach import StateMachine
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
        rospy.loginfo("GOING FOR COUNTDOWN! Go BUMBLEBEE GO!")        
        r = rospy.Rate(10)
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time) < self.sleep_time:
            r.sleep()
            #rospy.loginfo("%d Elapsed" % (rospy.get_time() - start_time))
        return 'succeeded'        

class Start(smach.State):
    def __init__(self, timeout, start_depth, start_heading=None):
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
        global global_heading
        
        rospy.loginfo("ENTERING Start state")
        
        if self.start_heading == None:
            self.start_heading = global_heading
            rospy.loginfo('No start heading provided; Taking current heading %s as start' % str(self.start_heading))
            
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
            resp = set_ConPIDMode(False, False, False, False, False, False, False)
            rospy.logdebug("PID and Mode is set")
        except rospy.ServiceException, e:
            rospy.logdebug("PID and Mode NOT set: %s" % e)

        #Reset DVL and Earth Odom here
#        drClient_DVL = dynamic_reconfigure.client.Client("WH_DVL")
#        drClient_DVL.update_configuration({"zero_distance":True})
        drClient_Earth = dynamic_reconfigure.client.Client("earth_odom")
        drClient_Earth.update_configuration({"zero_distance":True})

        locomotion_client.cancel_all_goals()
        #Setting PID (Fwd? Side? Head? Depth? Pitch?) and modes (Topside? Nav?)
        try:
            resp = set_ConPIDMode(True, True, True, True, True, False, False)
            rospy.logdebug("PID and Mode is set")
        except rospy.ServiceException, e:
            rospy.logdebug("PID and Mode NOT set: %s" % e)
        
        locomotion_client.send_goal(goal)
        locomotion_client.wait_for_result(rospy.Duration(self.timeout,0))
        rospy.loginfo("Dive Dive Dive!")
        
        locomotionGoal.depth_setpoint = self.start_depth
        locomotionGoal.heading_setpoint = self.start_heading
        locomotionGoal.sidemove_setpoint = 0
        locomotionGoal.forward_setpoint = 0
        return 'succeeded'

class End(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
                
    def execute(self,userdata):
        global set_ConPIDMode
        #Setting PID (Fwd? Side? Head? Depth? Pitch?) and modes (Topside? Nav?)
        try:
            resp = set_ConPIDMode(False, False, False, False, False, False, False)
            rospy.logdebug("PID and Mode is set")
        except rospy.ServiceException, e:
            rospy.logdebug("PID and Mode NOT set: %s" % e)
        return 'succeeded'

class HeadingCheck(smach.State):
    def __init__(self, min_head, max_head):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        self.min_head = min_head
        self.max_head = max_head
        
    def execute(self,userdata):

        global global_heading
        self.currHead = global_heading
        
        if self.currHead > 0 and self.currHead < 0.1:
            self.currHead = 359
        
        if self.currHead > self.max_head or self.currHead < self.min_head:
            rospy.loginfo('Currrent Heading is %s; NOT within %s to %s' % (str(self.currHead), str(self.min_head), str(self.max_head)))
            return 'failed'
        if self.currHead < self.max_head and self.currHead > self.min_head:
            rospy.loginfo('Currrent Heading is %s; WITHIn %s to %s' % (str(self.currHead), str(self.min_head), str(self.max_head)))
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

        rospy.loginfo("ENTERING GoToDistance state; locomotionGoal currently at depth %s and heading %s" 
                      % (str(locomotionGoal.depth_setpoint), str(locomotionGoal.heading_setpoint) ))
        
        if self.direction == 'fwd':

            #Setting Locomotion Mode (Forward, Sidemove) ; For Default, put both to False
            try:
                resp = set_LocoMode(False, False)
                rospy.logdebug("LocoMode set to Default")
            except rospy.ServiceException, e:
                rospy.logdebug("LocoMode Fwd NOT set: %s" % e)
            
            goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=self.distance,
                                                sidemove_setpoint=0,
                                                depth_setpoint=locomotionGoal.depth_setpoint,
                                                heading_setpoint=locomotionGoal.heading_setpoint)
        
        if self.direction == 'sway':
            
            #Setting Locomotion Mode (Forward, Sidemove) ; For Default, put both to False
            try:
                resp = set_LocoMode(False, True)
                rospy.logdebug("LocoMode set to Sidemove")
            except rospy.ServiceException, e:
                rospy.logdebug("LocoMode Sway NOT set: %s" % e)
                
            goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0,
                                                sidemove_setpoint=self.distance,
                                                depth_setpoint=locomotionGoal.depth_setpoint,
                                                heading_setpoint=locomotionGoal.heading_setpoint)

        rospy.loginfo('Going %s distance for %s m' % (str(self.direction), str(self.distance)))                                            
        locomotion_client.send_goal(goal)
        locomotion_client.wait_for_result(rospy.Duration(self.timeout,0))

        locomotionGoal.forward_setpoint = 0
        locomotionGoal.sidemove_setpoint = 0
             
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

        rospy.loginfo("ENTERING GoToDepth state; locomotionGoal currently at depth %s and heading %s" 
                      % (str(locomotionGoal.depth_setpoint), str(locomotionGoal.heading_setpoint) ))

        if self.depth == None:
            if locomotionGoal.depth_setpoint >= 0.5 :
                self.depth = locomotionGoal.depth_setpoint
            if locomotionGoal.depth_setpoint <0.5:
                if self.surface == True:
                    self.depth = locomotionGoal.depth_setpoint
                if self.surface == False:
                    self.depth = 0.5
                    rospy.loginfo("Task tried to bring vehicle to surface PREMATURELY; setting depth to 0.5")
        
        #seed goal
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0,
                                            sidemove_setpoint=0,
                                            depth_setpoint=self.depth,
                                            heading_setpoint=locomotionGoal.heading_setpoint)
        rospy.loginfo('Going to depth %s m' % str(self.depth))
        locomotion_client.send_goal(goal)
        locomotion_client.wait_for_result(rospy.Duration(self.timeout,0))
        
        #Updating locomotionGoal
        locomotionGoal.depth_setpoint = goal.depth_setpoint      
        locomotionGoal.forward_setpoint = 0
        locomotionGoal.sidemove_setpoint = 0
        
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

        rospy.loginfo("ENTERING GoToHeading state; locomotionGoal currently at depth %s and heading %s" 
                      % (str(locomotionGoal.depth_setpoint), str(locomotionGoal.heading_setpoint) ))

        if locomotionGoal.depth_setpoint < 0.5:
            locomotionGoal.depth_setpoint = 0.5
            rospy.loginfo("Task tried to bring vehicle to surface; setting depth to 0.5")
        
        #Setting Locomotion Mode (Forward, Sidemove) ; For Default, put both to False
        try:
            resp = set_LocoMode(False, False)
            rospy.logdebug("LocoMode set to Default")
        except rospy.ServiceException, e:
            rospy.logdebug("LocoMode Default NOT set: %s" % e)        
        
        if self.heading == None:
            self.heading = locomotionGoal.heading_setpoint
        
        if self.heading != None and self.relative == True:
            self.heading = self.normalize_angle(self.heading+locomotionGoal.heading_setpoint)

        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0,
                                            sidemove_setpoint=0,
                                            depth_setpoint=locomotionGoal.depth_setpoint,
                                            heading_setpoint=self.heading)

        rospy.loginfo('Going to heading %s deg' % str(self.heading))                                            
        locomotion_client.send_goal(goal)
        locomotion_client.wait_for_result(rospy.Duration(self.timeout,0))
        
        #Updating locomotionGoal
        locomotionGoal.heading_setpoint = goal.heading_setpoint
        locomotionGoal.forward_setpoint = 0
        locomotionGoal.sidemove_setpoint = 0
                     
        return 'succeeded'

    
class StoreGlobalCoord(smach.State):
    def __init__(self, task_name):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
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
       
        rospy.loginfo('Storing x=%s y=%s depth=%s heading=%s' % (str(global_x), str(global_y), str(global_depth), str(global_heading)))
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

        rospy.loginfo("ENTERING %s %s state; locomotionGoal currently at depth %s and heading %s" 
                      % (self.task_name, self.name, str(locomotionGoal.depth_setpoint), str(locomotionGoal.heading_setpoint) ))

        if locomotionGoal.depth_setpoint < 0.5:
            locomotionGoal.depth_setpoint = 0.5
            rospy.loginfo("Task tried to bring vehicle to surface; setting depth to 0.5")

        #Setting Locomotion Mode (Forward, Sidemove) ; For Default, put both to False
        try:
            resp = set_LocoMode(False, False)
            rospy.logdebug("LocoMode set to Default")
        except rospy.ServiceException, e:
            rospy.logdebug("LocoMode Default NOT set: %s" % e)   

        #connecting to task server
        if self.task_name != 'lane':
            rospy.logdebug('Mission attempt connecting to %s Server' % self.task_name)           
            rospy.wait_for_service(self.task_srv_name)   
            self.task_srv = rospy.ServiceProxy(self.task_srv_name, mission_to_vision)
            rospy.logdebug('Mission Connected to %s Server' % self.task_name)

        #Seeding goal
        goal = bbauv_msgs.msg.ControllerGoal()
        if self.start_heading == None: goal.heading_setpoint = locomotionGoal.heading_setpoint
        
        if self.start_heading != None: 
            goal.heading_setpoint = self.start_heading
            locomotionGoal.heading_setpoint = self.start_heading
            
        if self.start_depth == None: goal.depth_setpoint = locomotionGoal.depth_setpoint
        
        if self.start_depth != None: 
            goal.depth_setpoint = self.start_depth
            locomotionGoal.depth_setpoint = self.start_depth
            
        goal.forward_setpoint = 0
        goal.sidemove_setpoint = 0

        #Begin Searching For Task
        if self.task_name == 'lane':
            try:
                resp = lane_srv(True,locomotionGoal,self.use_left,self.num_lanes,False)
            except rospy.ServiceException, e:
                rospy.logdebug("Failed to start Search: %s" % e)
                return 'failed'  
        if self.task_name != 'lane':
            try:
                resp = self.task_srv(True, locomotionGoal, False)
                rospy.logdebug("Searching for %s" % self.task_name)  
            except rospy.ServiceException, e:
                rospy.logdebug("Failed to start Search: %s" % e)
                if self.task_srv != None:
                    rospy.logdebug('Closing connection to %s' % self.task_srv_name)
                    self.task_srv.close()
                return 'failed'

        r = rospy.Rate(30)
        start_time = rospy.get_time()
        rospy.loginfo("Hovering to search for %s" % self.task_name)
        while (not rospy.is_shutdown()) and ((rospy.get_time()-start_time) <= self.timeout):
            if not isSearchDone:
                #Hovering
                locomotion_client.send_goal(goal)
                locomotion_client.wait_for_result(rospy.Duration(1,0))
                                    
            if isSearchDone and caller_name == self.task_name :
                rospy.loginfo("Found %s. %d of %d secs elapsed" % (self.task_name, rospy.get_time()-start_time, self.timeout))
                if self.task_srv != None:
                    rospy.logdebug('Closing connection to %s' % self.task_srv_name)
                    self.task_srv.close()
                return 'succeeded'
            
            if isSearchFailed and caller_name == self.task_name :
                rospy.loginfo("Failed to find %s. %d of %d secs elapsed" % (self.task_name, rospy.get_time()-start_time, self.timeout))
                if self.task_srv != None:
                    rospy.logdebug('Closing connection to %s' % self.task_srv_name)
                    self.task_srv.close()
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
            rospy.logdebug("Search Motion Callback Complete")
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
        
        rospy.loginfo("ENTERING %s %s state; locomotionGoal currently at depth %s and heading %s" 
                      % (self.task_name, self.name, str(locomotionGoal.depth_setpoint), str(locomotionGoal.heading_setpoint) ))

        if locomotionGoal.depth_setpoint < 0.5:
            locomotionGoal.depth_setpoint = 0.5
            rospy.loginfo("Task tried to bring vehicle to surface; setting depth to 0.5")
        
        #Setting Locomotion Mode (Forward, Sidemove) ; For Default, put both to False
        try:
            resp = set_LocoMode(False, False)
            rospy.logdebug("LocoMode set to Default")
        except rospy.ServiceException, e:
            rospy.logdebug("LocoMode Default NOT set: %s" % e)
        
        #connecting to task server 

        if self.task_name != 'lane':
            rospy.loginfo('Mission attempt connecting to %s Server' % self.task_name)           
            rospy.wait_for_service(self.task_srv_name)   
            self.task_srv = rospy.ServiceProxy(self.task_srv_name, mission_to_vision)
            rospy.loginfo('Mission Connected to %s Server' % self.task_name)
        
        #Seeding goal
        goal = bbauv_msgs.msg.ControllerGoal()
        if self.start_heading == None: goal.heading_setpoint = locomotionGoal.heading_setpoint
        
        if self.start_heading != None: 
            goal.heading_setpoint = self.start_heading
            locomotionGoal.heading_setpoint = self.start_heading
            
        if self.start_depth == None: goal.depth_setpoint = locomotionGoal.depth_setpoint
        
        if self.start_depth != None: 
            goal.depth_setpoint = self.start_depth
            locomotionGoal.depth_setpoint = self.start_depth
                
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
                if self.task_srv != None:
                    rospy.logdebug('Closing connection to %s' % self.task_srv_name)
                    self.task_srv.close()
                return 'failed'  
        
        #Begin Moving Around while searching
        if self.direction == 'fwd':
            goal.forward_setpoint = self.distance
            goal.sidemove_setpoint = 0
            locomotion_client.send_goal(goal, self.motion_callback)
   
        if self.direction == 'sway':

            #Setting Locomotion Mode (Forward, Sidemove) ; For Default, put both to False
            try:
                resp = set_LocoMode(False, True)
                rospy.logdebug("LocoMode set to Sidemove")
            except rospy.ServiceException, e:
                rospy.logdebug("LocoMode Fwd NOT set: %s" % e)
                            
            goal.forward_setpoint = 0
            goal.sidemove_setpoint = self.distance
            locomotion_client.send_goal(goal, self.motion_callback)    
        
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        rospy.loginfo("Moving %s %s meters to search for %s" % (self.direction, str(self.distance) ,self.task_name))
        while (not rospy.is_shutdown()) and ((rospy.get_time()-start_time) <= self.timeout):
            
            if not isSearchDone:
                if self.motionStatus:
                    if self.task_name == 'lane':
                        try:
                            rospy.loginfo('Search Motion Complete: Failed to find %s' % self.task_name)
                            resp = lane_srv(False, locomotionGoal, False, 1, True)            
                        except rospy.ServiceException, e:
                            rospy.loginfo("Search Motion Complete: Failed to abort: %s" % e)    
                            return 'failed'                         

                    if self.task_name != 'lane':
                        try:
                            rospy.loginfo('Search Motion Complete: Failed to find %s' % self.task_name)
                            resp = self.task_srv(False, locomotionGoal, True)            

                        except rospy.ServiceException, e:
                            rospy.loginfo("Search Motion Complete: Failed to abort: %s" % e)
                            if self.task_srv != None:
                                rospy.logdebug('Closing connection to %s' % self.task_srv_name)
                                self.task_srv.close()
                            return 'failed'

                    if self.task_srv != None:
                        rospy.logdebug('Closing connection to %s' % self.task_srv_name)
                        self.task_srv.close()
                    return 'failed'
                                    
            if isSearchDone and caller_name == self.task_name:
                locomotionGoal.depth_setpoint = goal.depth_setpoint 
                locomotionGoal.heading_setpoint = goal.heading_setpoint
                locomotion_client.cancel_all_goals()             
                rospy.loginfo("Found %s. %d of %d secs elapsed" % (self.task_name, rospy.get_time()-start_time, self.timeout))
                if self.task_srv != None:
                    rospy.logdebug('Closing connection to %s' % self.task_srv_name)
                    self.task_srv.close()
                return 'succeeded'
            
            if isSearchFailed and caller_name == self.task_name:
                locomotionGoal.depth_setpoint = goal.depth_setpoint 
                locomotionGoal.heading_setpoint = goal.heading_setpoint
                locomotion_client.cancel_all_goals()
                rospy.loginfo("Failed to find %s. %d of %d secs elapsed" % (self.task_name, rospy.get_time()-start_time, self.timeout))
                if self.task_srv != None:
                    rospy.logdebug('Closing connection to %s' % self.task_srv_name)
                    self.task_srv.close()
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
                if self.task_srv != None:
                    rospy.logdebug('Closing connection to %s' % self.task_srv_name)
                    self.task_srv.close() 
                return 'failed'              
            except rospy.ServiceException, e:
                rospy.loginfo("Timed Out: Failed to abort: %s" % e)
                if self.task_srv != None:
                    rospy.logdebug('Closing connection to %s' % self.task_srv_name)
                    self.task_srv.close()
                return 'failed'


class WaitOut(smach.State):
    def __init__(self, task_name, timeout, boundLength=999, depth_limit=15):
                
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.name = self.__class__.__name__ 
        self.task_name = task_name
        self.task_srv_name = task_name + '_srv'
        self.task_srv = None
        self.timeout = timeout

        self.depth_limit = depth_limit        
        self.firstX = None
        self.firstY = None
                
        self.boundLength = boundLength
        self.isOutofBound = False

    def boundingBoxCheck(self, firstX, firstY, depth_limit):
        global global_x
        global global_y
        global global_depth
        
        #checks if the AUV has maneuvered out of a box of size
#        rospy.logdebug('Checking Bounding Box while waiting for task to complete')
        if global_x > (firstX+self.boundLength) or global_x < (firstX-self.boundLength) or global_y > (firstY+self.boundLength) or global_y < (firstY-self.boundLength):
            self.isOutofBound = True
            rospy.loginfo("Vehicle moving out of bounding box length %s centered at x=%s y=%s" % (str(self.boundLength), str(firstX), str(firstY)))
        if global_depth > depth_limit:
            self.isOutofBound = True
            rospy.loginfo("Vehicle tried to dive to %s when limit is %s" % (str(global_depth), str(depth_limit)))
                
    def execute(self, userdata):
        global lane_srv
        global locomotion_client
        global locomotionGoal
        global isTaskComplete
        global isTaskFailed
        isTaskFailed = False
        isTaskComplete = False     
           
        rospy.loginfo("ENTERING %s %s state; locomotionGoal currently at depth %s and heading %s" 
                      % (self.task_name, self.name, str(locomotionGoal.depth_setpoint), str(locomotionGoal.heading_setpoint) ))

        if locomotionGoal.depth_setpoint < 0.5:
            locomotionGoal.depth_setpoint = 0.5
            rospy.loginfo("Task tried to bring vehicle to surface; setting depth to 0.5")

        #connecting to task server;      
        if self.task_name != 'lane':        
            #connecting to task server
            rospy.loginfo('Mission Attempt to Connect to %s Server' % self.task_name)        
            rospy.wait_for_service(self.task_srv_name)
            self.task_srv = rospy.ServiceProxy(self.task_srv_name, mission_to_vision)
            rospy.loginfo('Mission Connected to %s Server' % self.task_name)        
        
        #Storing Task found coordinates        
        self.firstX = global_x
        self.firstY = global_y
        
        #Waiting Out
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        rospy.loginfo("%s: Found. Task Controlling Vehicle" % (self.task_name))
        while (not rospy.is_shutdown()) and ((rospy.get_time()-start_time) <= self.timeout):
            self.boundingBoxCheck(self.firstX, self.firstY, self.depth_limit)
#            rospy.loginfo('global_x = %s global_depth = %s' % (str(global_x), str(global_depth)))
            if self.isOutofBound:
                return 'failed'

            if isTaskComplete and caller_name == self.task_name:               
                rospy.loginfo("Completed %s. %d of %d secs elapsed" % (self.task_name, rospy.get_time()-start_time, self.timeout))
                if self.task_srv != None:
                    rospy.logdebug('Closing connection to %s' % self.task_srv_name)
                    self.task_srv.close()
                return 'succeeded'
            if isTaskFailed  and caller_name == self.task_name:              
                rospy.loginfo("Failed to complete %s. %d of %d secs elapsed" % (self.task_name, rospy.get_time()-start_time, self.timeout))
                if self.task_srv != None:
                    rospy.logdebug('Closing connection to %s' % self.task_srv_name)
                    self.task_srv.close()
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
                if self.task_srv != None:
                    rospy.logdebug('Closing connection to %s' % self.task_srv_name)
                    self.task_srv.close() 
                return 'failed'              
            except rospy.ServiceException, e:
                rospy.loginfo("Timed Out: Failed to abort: %s" % e)       
                if self.task_srv != None:
                    rospy.logdebug('Closing connection to %s' % self.task_srv_name)
                    self.task_srv.close()     
                return 'failed'

class Nav(smach.State):
    
    def __init__ (self, prep_timeout, nav_timeout, end_timeout, x=0, y=0, start_depth = 0.5, end_depth=0.5, end_heading=0, place= None): #yaw here is BBAUV's NED convention'
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
        
        rospy.loginfo('global_x = %s global_y= %s' % (str(global_x), str(global_y)))

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
        
        rospy.loginfo('Distance = %s', str(distance))
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

        rospy.loginfo("ENTERING %s state; locomotionGoal currently at depth %s and heading %s" 
                      % (self.name, str(locomotionGoal.depth_setpoint), str(locomotionGoal.heading_setpoint) ))

        if locomotionGoal.depth_setpoint < 0.5:
            locomotionGoal.depth_setpoint = 0.5
            rospy.loginfo("Task tried to bring vehicle to surface; setting depth to 0.5")
        
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
                rospy.logdebug("LocoMode set to Default by Navigation State")
            except rospy.ServiceException, e:
                rospy.logdebug("LocoMode Default NOT set: %s" % e) 
    
            #Setting PID (Fwd? Side? Head? Depth? Pitch?) and modes (Topside? Nav?)
            try:
                resp = set_ConPIDMode(True, True, True, True, True, False, False)
                rospy.logdebug("PID and Mode is set")
            except rospy.ServiceException, e:
                rospy.logdebug("PID and Mode NOT set: %s" % e)
          
                
            #Change Depth and Face Heading First

            self.start_heading = self.start_heading_gen(self.x, self.y, global_x, global_y)

            heading_goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0,sidemove_setpoint=0,depth_setpoint=self.start_depth,heading_setpoint=self.start_heading)
            locomotion_client.send_goal(heading_goal) 
            rospy.loginfo('Starting Navigation from depth %s m and facing yaw=%s deg' % (str(self.start_depth), str(self.start_heading)))
            locomotion_client.wait_for_result(rospy.Duration(self.prep_timeout,0))
            
            #Navigating to global coord       
            
            self.start_heading = self.start_heading_gen(self.x, self.y, global_x, global_y)
            self.distanceToCoord = self.distanceToGlobalCoord(self.x, self.y, global_x, global_y)

            nav_goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=self.distanceToCoord,sidemove_setpoint=0,depth_setpoint=self.start_depth,heading_setpoint=self.start_heading)

            try:
                resp = set_LocoMode(True, False)
                rospy.logdebug("LocoMode set to Forward by Navigation State")
            except rospy.ServiceException, e:
                rospy.logdebug("LocoMode Default NOT set: %s" % e) 

            #rospy.loginfo('FOR HOANG: Heading Setpoint at %s', str(self.start_heading))    
            locomotion_client.send_goal(nav_goal)
            rospy.loginfo('Navigating to x=%s y=%s.' % (str(self.x), str(self.y)))
            locomotion_client.wait_for_result(rospy.Duration(self.nav_timeout,0))

            #Final Adjustments to make sure we're there accurately

            self.start_heading = self.start_heading_gen(self.x, self.y, global_x, global_y)

            finalheading_goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0,sidemove_setpoint=0,depth_setpoint=self.start_depth,heading_setpoint=self.start_heading)

            try:
                resp = set_LocoMode(False, False)
                rospy.logdebug("LocoMode set to Default by Navigation State")
            except rospy.ServiceException, e:
                rospy.logdebug("LocoMode Default NOT set: %s" % e) 

            locomotion_client.send_goal(finalheading_goal)
            rospy.logdebug('Navigation Final Turn.')
            locomotion_client.wait_for_result(rospy.Duration(10,0))

            self.start_heading = self.start_heading_gen(self.x, self.y, global_x, global_y)
            self.distanceToCoord = self.distanceToGlobalCoord(self.x, self.y, global_x, global_y)

            finalnav_goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=self.distanceToCoord,sidemove_setpoint=0,depth_setpoint=self.start_depth,heading_setpoint=self.start_heading)

            locomotion_client.send_goal(finalnav_goal)
            rospy.logdebug('Navigation Final Move.')
            locomotion_client.wait_for_result(rospy.Duration(10,0))
            
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
                rospy.logdebug("LocoMode set to Default")
            except rospy.ServiceException, e:
                rospy.logdebug("LocoMode Default NOT set: %s" % e) 
    
            #Setting PID (Fwd? Side? Head? Depth? Pitch?) and modes (Topside? Nav?)
            try:
                resp = set_ConPIDMode(True, True, True, True, False, False, True)
                rospy.logdebug("PID and Mode is set")
            except rospy.ServiceException, e:
                rospy.logdebug("PID and Mode NOT set: %s" % e)
                            
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
                rospy.logdebug("PID and Mode is set")
            except rospy.ServiceException, e:
                rospy.logdebug("PID and Mode NOT set: %s" % e)
        
        #End Navigation with the following heading and depth  
        end_goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0,sidemove_setpoint=0,depth_setpoint=self.end_depth,heading_setpoint=self.end_heading)
        locomotion_client.send_goal(end_goal)
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
    global task_id
    global caller_name
    global task_list
    
    search_call = False
    task_call = False
    
    if "id" in req._connection_header:
        task_id = str(req._connection_header['id'])
        rospy.logdebug('task_id = %s' % task_id)

        caller_name = task_list[task_id]
        rospy.loginfo('Service call made by %s to mission server with search: %s task: %s' 
            % (caller_name +'_srv', req.search_request, req.task_complete_request))
    
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
        #locomotionGoal = req.task_complete_ctrl 
    
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
caller_name = None
marker_id = 1
task_list = {'0':'lane','1':'traffic', '2':'park','3':'speedtrap','4':'tollbooth','5':'drivethru','6':'acoustic'}

if __name__ == '__main__':

#    rospy.init_node('Mission_planner', log_level=rospy.INFO, anonymous=True)
    rospy.init_node('Mission_planner', log_level=rospy.DEBUG, anonymous=True)

    test_mode = False
    
    mission_server = rospy.Service('mission_srv', vision_to_mission, handle_srv)
    rospy.loginfo('MissionServer Initialized!')

    raw_input('''

    Pre Launch

    KillSwitch : Thomas
    Launch : Eng Wei
    Depth Offset : Eng Wei
    Camera Settings : Jon
    Telemetry Check : Eng Wei
    Load Ammo : Thomas
    Acoustics : Louis
    Thresholds: Everyone
    GUI DOWN : Everyone
    Vehicle Position and Dummy Plug : Thomas
    rosnode cleanup : TC
    task.launch : TC

    >>> Press ENTER for connection test...
    ''')

    if not test_mode:    
        # Subscribing to Telemetry
        odom_sub = rospy.Subscriber('/earth_odom', Odometry, odomCallback)
        depth_sub = rospy.Subscriber('/depth', depth, depthCallback)
        AHRS_sub = rospy.Subscriber('/AHRS8_data_e', imu_data, AHRSCallback)
        Altitude_sub = rospy.Subscriber('/altitude', Float32, AltitudeCallback)
         
         # Testing Task services

        for key,value in task_list.iteritems():
            test_srv = value + '_srv'
            rospy.logdebug('Mission Waiting for %s to start up...' % test_srv)
            rospy.wait_for_service(test_srv)
            testing_srv = rospy.ServiceProxy(test_srv , mission_to_lane)
            rospy.loginfo('Mission Connected to %s' % test_srv)
            testing_srv.close()

        
        rospy.loginfo('Mission Waiting for LaneServer to start up...')
        rospy.wait_for_service('lane_srv')
        lane_srv = rospy.ServiceProxy('lane_srv', mission_to_lane)
        rospy.loginfo('Mission Connected to LaneServer')
          
        # Action Client for PIDs
        locomotion_client = actionlib.SimpleActionClient('LocomotionServer', bbauv_msgs.msg.ControllerAction)
        #rospy.loginfo('Mission Waiting for Locomotion Server to start up...')
        #locomotion_client.wait_for_server()
        rospy.loginfo("Mission connected to LocomotionServer")
          
        #Service Client for PID & Modes
        #rospy.loginfo('Mission Waiting for Set Controller Service to start up...')
        #rospy.wait_for_service('set_controller_srv')
        set_ConPIDMode = rospy.ServiceProxy('set_controller_srv', set_controller)
        rospy.loginfo('Mission Connected to Set Controller Service')
        
        #rospy.loginfo('Mission Waiting for Locomotion Modes Service to start up...')
        #rospy.wait_for_service('locomotion_mode_srv')
        set_LocoMode = rospy.ServiceProxy('locomotion_mode_srv',locomotion_mode)
        rospy.loginfo('Mission Connected to Locomotion Mode Service')
        
        # Action Client for Move Base
        movebase_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        #rospy.loginfo('Mission Waiting for Move Base Service to start up...')
        #movebase_client.wait_for_server()
        rospy.loginfo("Mission connected to MovebaseServer")

        raw_input('''
        Connection OK!
        >>> Press ENTER to continue mission...
        ''')
	
### Insert Mission Here ###
 
    sm_mission = StateMachine(outcomes=['mission_complete','mission_failed'])
    
    with sm_mission:
    
#DEBUGS
#        StateMachine.add('DEPTHCHANGE', GoToDepth(15,3), transitions={'succeeded':'TASK_EXECUTION'})
        StateMachine.add('COUNTDOWN', Countdown(10), transitions={'succeeded':'START'})
#        StateMachine.add('COUNTDOWN', Countdown(0), transitions={'succeeded':'LANE_GATE'})        

        #Competition Side
        StateMachine.add('START',Start(5,0.6), transitions={'succeeded':'TURN_TO_GATE'})

        #Debug
#        StateMachine.add('START',Start(0.5,0.6,320), transitions={'succeeded':'LANE_PARK'}) #320 good for ping
        
        StateMachine.add('TURN_TO_GATE', GoToHeading(3), transitions={'succeeded':'GO_TO_GATE'}) #practice side is 295, comp side is 40
        StateMachine.add('GO_TO_GATE', GoToDistance(70, 9, 'fwd'), transitions={'succeeded':'LANE_GATE'})
        
###################################################################        
        lane_gate = StateMachine(outcomes=['lane_complete', 'lane_failed'])
            
        with lane_gate:

            StateMachine.add('LOOK_FWD', LinearSearch('lane', 30, 5, 'fwd', False, 1), transitions={'succeeded':'TASK_EXECUTION', 'failed':'LOOK_LEFT'})
            StateMachine.add('LOOK_LEFT', LinearSearch('lane', 20, -1.5, 'sway', False, 1), transitions={'succeeded':'TASK_EXECUTION', 'failed':'LOOK_BEHIND'})
            StateMachine.add('LOOK_BEHIND', LinearSearch('lane', 20, -2, 'fwd', False, 1), transitions={'succeeded':'TASK_EXECUTION', 'failed':'LOOK_RIGHT'})
            StateMachine.add('LOOK_RIGHT', LinearSearch('lane', 40, 3, 'sway', False, 1), transitions={'succeeded':'TASK_EXECUTION', 'failed':'LOOK_FWD2'})
            StateMachine.add('LOOK_FWD2', LinearSearch('lane', 20, 2.5, 'fwd', False, 1), transitions={'succeeded':'TASK_EXECUTION', 'failed':'LOOK_LEFT3'})
            StateMachine.add('LOOK_LEFT3', LinearSearch('lane', 20, -1.5, 'sway', False, 1), transitions={'succeeded':'TASK_EXECUTION', 'failed':'lane_failed'})

            task = {'tN':'lane', 'tOut':40, 'bL':3}
            task_execution = Sequence(outcomes=['succeeded', 'failed'], connector_outcome = 'succeeded')
            with task_execution:
                Sequence.add('STORE_FOUND', StoreGlobalCoord('mission_laneGate_found'))
                Sequence.add(task['tN'], WaitOut(task['tN'], task['tOut'], task['bL']))
                Sequence.add('STORE_DONE', StoreGlobalCoord('mission_laneGate_done'))                               
            StateMachine.add('TASK_EXECUTION', task_execution, transitions={'succeeded':'HEADINGCHANGE', 'failed':'lane_failed'})

            StateMachine.add('HEADINGCHANGE', GoToHeading(10), transitions={'succeeded':'lane_complete'})
                                        
        StateMachine.add('LANE_GATE', lane_gate, transitions={'lane_complete':'TRAFFIC', 'lane_failed':'FAIL_LANEGATE_HEADING'})

        StateMachine.add('FAIL_LANEGATE_HEADING', GoToHeading(20, 30), transitions={'succeeded':'TRAFFIC'})
        
###################################################################        
        traffic = StateMachine(outcomes=['traffic_complete', 'traffic_failed'])
        
        with traffic:
        
            StateMachine.add('DEPTHCHANGE', GoToDepth(15,3), transitions={'succeeded':'GOFWD'})
            StateMachine.add('GOFWD', GoToDistance(30,3,'fwd'), transitions={'succeeded':'LOOKFWD'})
            StateMachine.add('LOOKFWD', LinearSearch('traffic', 30, 3, 'fwd'), transitions={'succeeded':'TASK_EXECUTION', 'failed':'ZIGZAGSEARCH'})

            #Zig Zag Search
            zP = {'tN':'traffic', 'fTo':15, 'sTo': 30, 
                  'fD':1, 'sD': 5, 
                  'isL':False , 'nL': 1}
            zigzagSearch = Sequence(outcomes=['succeeded', 'failed'], connector_outcome= 'failed')
            with zigzagSearch:
                #Initial
                Sequence.add('RIGHT_START', LinearSearch(zP['tN'], zP['sTo'], zP['sD']/float(2) , 'sway', zP['isL'], zP['nL']))
                Sequence.add('FWD', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                Sequence.add('LEFT', LinearSearch(zP['tN'], zP['sTo'], -1*zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('FWD2', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                #Final
                Sequence.add('RIGHT_FINAL', LinearSearch(zP['tN'], zP['sTo'], zP['sD']/float(2) , 'sway', zP['isL'], zP['nL']))

            StateMachine.add('ZIGZAGSEARCH', zigzagSearch, transitions={'succeeded':'TASK_EXECUTION', 'failed':'traffic_failed'})            

            task = {'tN':'traffic', 'tOut':100, 'bL':7}
            task_execution = Sequence(outcomes=['succeeded', 'failed'], connector_outcome = 'succeeded')
            with task_execution:
                Sequence.add('STORE_FOUND', StoreGlobalCoord('mission_traffic_found'))
                Sequence.add(task['tN'], WaitOut(task['tN'], task['tOut'], task['bL']))
                Sequence.add('STORE_DONE', StoreGlobalCoord('mission_traffic_done'))                               
            StateMachine.add('TASK_EXECUTION', task_execution, transitions={'succeeded':'traffic_complete', 'failed':'traffic_failed'})
            
        StateMachine.add('TRAFFIC', traffic, transitions={'traffic_complete':'LANE_TRAFFIC', 'traffic_failed':'DRIVE_THRU'})  
                  
###################################################################
        lane_traffic = StateMachine(outcomes=['lane_complete', 'lane_failed'])
        
        with lane_traffic:
            
            StateMachine.add('DEPTHCHANGE', GoToDepth(15,0.6), transitions={'succeeded':'GOLEFT'})
            StateMachine.add('GOLEFT', GoToDistance(30, -1, 'sway'), transitions={'succeeded':'GOFWD'})
            StateMachine.add('GOFWD', GoToDistance(20, 3, 'fwd'), transitions={'succeeded':'HOVER'})
            StateMachine.add('HOVER', HoverSearch('lane', 10, False, 1), transitions={'succeeded':'TASK_EXECUTION', 'failed':'LOOK_FWD'})
            StateMachine.add('LOOK_FWD', LinearSearch('lane', 20, 1, 'fwd', False, 1), transitions={'succeeded':'TASK_EXECUTION', 'failed':'LOOK_LEFT'})
            StateMachine.add('LOOK_LEFT', LinearSearch('lane', 20, -2, 'sway', False, 1), transitions={'succeeded':'TASK_EXECUTION', 'failed':'LOOK_RIGHT'})
            StateMachine.add('LOOK_RIGHT', LinearSearch('lane', 20, 4, 'sway', False, 1), transitions={'succeeded':'TASK_EXECUTION', 'failed':'LOOK_BEHIND'})
            StateMachine.add('LOOK_BEHIND', LinearSearch('lane', 20, -2, 'fwd', False, 1), transitions={'succeeded':'TASK_EXECUTION', 'failed':'LOOK_LEFT2'})
            StateMachine.add('LOOK_LEFT2', LinearSearch('lane', 20, -2, 'sway', False, 1), transitions={'succeeded':'TASK_EXECUTION', 'failed':'LOOK_RIGHT2'})
            StateMachine.add('LOOK_RIGHT2', LinearSearch('lane', 20, 4, 'sway', False, 1), transitions={'succeeded':'TASK_EXECUTION', 'failed':'lane_failed'})

            task = {'tN':'lane', 'tOut':60, 'bL':3}
            task_execution = Sequence(outcomes=['succeeded', 'failed'], connector_outcome = 'succeeded')
            with task_execution:
                Sequence.add('STORE_FOUND', StoreGlobalCoord('mission_laneTraffic_found'))
                Sequence.add(task['tN'], WaitOut(task['tN'], task['tOut'], task['bL']))
                Sequence.add('STORE_DONE', StoreGlobalCoord('mission_laneTraffic_done'))                               
            StateMachine.add('TASK_EXECUTION', task_execution, transitions={'succeeded':'HEADINGCHANGE', 'failed':'lane_failed'})

            StateMachine.add('HEADINGCHANGE', GoToHeading(20), transitions={'succeeded':'CHECK_HEADING'})    
            StateMachine.add('CHECK_HEADING', HeadingCheck(300,360), transitions = {'succeeded':'lane_complete', 'failed':'HEADINGCHANGE2'})
            StateMachine.add('HEADINGCHANGE2', GoToHeading(20, 330), transitions={'succeeded':'lane_complete'})

        StateMachine.add('LANE_TRAFFIC', lane_traffic, transitions = {'lane_complete':'TOLLBOOTH', 'lane_failed':'DRIVE_THRU'})  

###################################################################
        
        tollbooth = StateMachine(outcomes=['toll_complete', 'toll_failed'])
        with tollbooth:
                   
            StateMachine.add('DEPTHCHANGE', GoToDepth(15,3), transitions={'succeeded':'GOFWD'}) 
            StateMachine.add('GOFWD', GoToDistance(60, 10, 'fwd'), transitions={'succeeded':'SEARCHFWD'})
            StateMachine.add('SEARCHFWD', LinearSearch('tollbooth', 30 ,5,'fwd'), transitions={'succeeded':'TASK_EXECUTION', 'failed':'ZIGZAGSEARCH'})

            #Zig Zag Search
            zP = {'tN':'tollbooth', 'fTo':20, 'sTo': 60, 
                  'fD':1, 'sD': 7, 
                  'isL':False , 'nL': 1}
            zigzagSearch = Sequence(outcomes=['succeeded', 'failed'], connector_outcome= 'failed')
            with zigzagSearch:
                #Initial
                Sequence.add('LEFT_START', LinearSearch(zP['tN'], zP['sTo'], -1*zP['sD']/float(2) , 'sway', zP['isL'], zP['nL']))
                Sequence.add('RIGHT1', LinearSearch(zP['tN'], zP['sTo'], zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('FWD1', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                Sequence.add('LEFT2', LinearSearch(zP['tN'], zP['sTo'], -1*zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('FWD2', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                Sequence.add('RIGHT2', LinearSearch(zP['tN'], zP['sTo'], zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('FWD3', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                Sequence.add('LEFT3', LinearSearch(zP['tN'], zP['sTo'], -1*zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('FWD4', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                #Final
                Sequence.add('RIGHT_FINAL', LinearSearch(zP['tN'], zP['sTo'], zP['sD']/float(2) , 'sway', zP['isL'], zP['nL']))
            StateMachine.add('ZIGZAGSEARCH', zigzagSearch, transitions={'succeeded':'TASK_EXECUTION', 'failed':'toll_failed'})
            
            task = {'tN':'tollbooth', 'tOut':180, 'bL':5}
            task_execution = Sequence(outcomes=['succeeded', 'failed'], connector_outcome = 'succeeded')
            with task_execution:
                Sequence.add('STORE_FOUND', StoreGlobalCoord('mission_toll_found'))
                Sequence.add(task['tN'], WaitOut(task['tN'], task['tOut'], task['bL']))
                Sequence.add('STORE_DONE', StoreGlobalCoord('mission_toll_done'))                             
            StateMachine.add('TASK_EXECUTION', task_execution, transitions={'succeeded':'toll_complete', 'failed':'toll_failed'})            
            
        StateMachine.add('TOLLBOOTH', tollbooth, transitions={'toll_complete':'SPEEDTRAP', 'toll_failed':'DRIVE_THRU'})

###################################################################

        speed = StateMachine(outcomes=['speed_complete', 'speed_failed'])
        with speed:
            StateMachine.add('DEPTHCHANGE', GoToDepth(15,0.6), transitions={'succeeded':'ZIGZAGSEARCH'})

            #Zig Zag Search
            zP = {'tN':'speedtrap', 'fTo':20, 'sTo': 60, 
                  'fD':-1, 'sD': 7, 
                  'isL':False , 'nL': 1}
            zigzagSearch = Sequence(outcomes=['succeeded', 'failed'], connector_outcome= 'failed')
            with zigzagSearch:
                #Initial
                Sequence.add('LEFT_START', LinearSearch(zP['tN'], zP['sTo'], -1*zP['sD']/float(2) , 'sway', zP['isL'], zP['nL']))
                Sequence.add('BACK1', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                Sequence.add('RIGHT2', LinearSearch(zP['tN'], zP['sTo'], zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('BACK2', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                Sequence.add('LEFT2', LinearSearch(zP['tN'], zP['sTo'], -1*zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('BACK3', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                Sequence.add('RIGHT3', LinearSearch(zP['tN'], zP['sTo'], zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('BACK4', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                #Final
                Sequence.add('LEFT_FINAL', LinearSearch(zP['tN'], zP['sTo'], -1*zP['sD']/float(2) , 'sway', zP['isL'], zP['nL']))
            StateMachine.add('ZIGZAGSEARCH', zigzagSearch, transitions={'succeeded':'TASK_EXECUTION', 'failed':'speed_failed'})  
            
            task = {'tN':'speedtrap', 'tOut':180, 'bL':5}
            task_execution = Sequence(outcomes=['succeeded', 'failed'], connector_outcome = 'succeeded')
            with task_execution:
                Sequence.add('STORE_FOUND', StoreGlobalCoord('mission_speed_found'))
                Sequence.add(task['tN'], WaitOut(task['tN'], task['tOut'], task['bL']))
                Sequence.add('STORE_DONE', StoreGlobalCoord('mission_speed_done'))                             
            StateMachine.add('TASK_EXECUTION', task_execution, transitions={'succeeded':'speed_complete', 'failed':'speed_failed'})            
                      
        StateMachine.add('SPEEDTRAP', speed, transitions={'speed_complete':'DRIVE_THRU', 'speed_failed':'DRIVE_THRU'})            
        
###################################################################        
        #EDIT LISTENING POST COORD HERE!!!!!!!!
#        StateMachine.add('NAVTO_LISTENING_POST', Nav(30,60,30,40,10,0.6,0.6,350), transitions = {'succeeded':'DRIVE_THRU', 'failed':'DRIVE_THRU'})        

###################################################################

        drive = StateMachine(outcomes=['drive_complete', 'drive_failed'])
        with drive:            
        
            StateMachine.add('DEPTHCHANGE', GoToDepth(15,1), transitions={'succeeded':'HEADINGCHANGE'})
            StateMachine.add('HEADINGCHANGE', GoToHeading(20,320), transitions={'succeeded':'HOVER'})
            StateMachine.add('HOVER', HoverSearch('acoustic', 60), transitions={'succeeded':'STORE_FOUND', 'failed':'GOFWD'})
            StateMachine.add('GOFWD', GoToDistance(60, 10, 'fwd'), transitions={'succeeded':'HOVER2'})
            StateMachine.add('HOVER2', HoverSearch('acoustic', 60), transitions={'succeeded':'STORE_FOUND', 'failed':'GOFWD2'})
            StateMachine.add('GOFWD2', GoToDistance(60, 10, 'fwd'), transitions={'succeeded':'HOVER3'})
            StateMachine.add('HOVER3', HoverSearch('acoustic', 60), transitions={'succeeded':'STORE_FOUND', 'failed':'GOFWD3'})
            StateMachine.add('GOFWD3', GoToDistance(60, 10, 'fwd'), transitions={'succeeded':'HOVER4'})
            StateMachine.add('HOVER4', HoverSearch('acoustic', 60), transitions={'succeeded':'STORE_FOUND', 'failed':'drive_failed'})

            StateMachine.add('STORE_FOUND', StoreGlobalCoord('mission_pinger_found'), transitions={'succeeded':'PINGER', 'failed':'PINGER'})            
            StateMachine.add('PINGER', WaitOut('acoustic', 240, 200), transitions={'succeeded':'STORE_DONE', 'failed':'STORE_DONE'})
            StateMachine.add('STORE_DONE', StoreGlobalCoord('mission_pinger_done'), transitions={'succeeded':'HOVER5', 'failed':'HOVER5'})
            
            StateMachine.add('HOVER5', HoverSearch('drivethru', 20), transitions={'succeeded':'PICKUP', 'failed':'SEARCH_LEFT'})    
            StateMachine.add('SEARCH_LEFT', LinearSearch('drivethru', 20, -2, 'sway'), transitions={'succeeded':'PICKUP', 'failed':'SEARCH_RIGHT'})
            StateMachine.add('SEARCH_RIGHT', LinearSearch('drivethru', 40, 4, 'sway'), transitions={'succeeded':'PICKUP', 'failed':'SEARCH_LEFT2'})
            StateMachine.add('SEARCH_LEFT2', LinearSearch('drivethru', 20, -2, 'sway'), transitions={'succeeded':'PICKUP', 'failed':'SEARCH_FRONT'})
            StateMachine.add('SEARCH_FRONT', LinearSearch('drivethru', 20, 2, 'fwd'), transitions={'succeeded':'PICKUP', 'failed':'SEARCH_REAR'})
            StateMachine.add('SEARCH_REAR', LinearSearch('drivethru', 20, -2, 'fwd'), transitions={'succeeded':'PICKUP', 'failed':'drive_failed'})            

            StateMachine.add('PICKUP', WaitOut('drivethru', 120, 50), transitions={'succeeded':'SURFACE', 'failed':'drive_failed'})
            StateMachine.add('SURFACE', GoToDepth(15, 0.2, surface = True), transitions={'succeeded':'DIVE_AGAIN'})
            StateMachine.add('DIVE_AGAIN', GoToDepth(15, 1), transitions={'succeeded':'HOVER6'})
            StateMachine.add('HOVER6', HoverSearch('acoustic', 60), transitions={'succeeded':'PINGER2', 'failed':'SHORTHOVER'})
            StateMachine.add('PINGER2', WaitOut('acoustic', 240, 50), transitions={'succeeded':'SHORTHOVER', 'failed':'NAVTO_PING1'})

            StateMachine.add('NAVTO_PING1', Nav(30,60,30,1, place = 'mission_pinger_done'), transitions={'succeeded':'SHORTHOVER', 'failed':'SHORTHOVER'})
            StateMachine.add('SHORTHOVER', HoverSearch('drivethru', 0.5), transitions={'succeeded':'DROPIT', 'failed':'DROPIT'})    
            StateMachine.add('DROPIT', WaitOut('drivethru', 30, 20), transitions={'succeeded':'drive_complete', 'failed':'drive_failed'})

        StateMachine.add('DRIVE_THRU', drive, transitions = {'drive_complete':'SURFACE_HAPPY', 'drive_failed':'SURFACE_SAD'})

###################################################################        
        #Mission Ending
        StateMachine.add('SURFACE_SAD', GoToDepth(15, 0, surface=True), transitions={'succeeded':'END_SAD'})
        StateMachine.add('SURFACE_HAPPY', GoToDepth(15, -1, surface=True), transitions={'succeeded':'END_HAPPY'})        
        StateMachine.add('END_SAD', End(), transitions={'succeeded':'mission_failed'})
        StateMachine.add('END_HAPPY', End(), transitions={'succeeded':'mission_complete'})

### Mission Ends Here ###  

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('mission_server', sm_mission, '/MISSION_PLANNER')
    sis.start()

    try:
        outcome = sm_mission.execute()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

########################



