#!/usr/bin/env python2

'''
Created on Apr 25, 2013

@author: gohew
'''
# ROS and System libraries
import roslib; roslib.load_manifest('Vision')
import rospy
import actionlib

import sys
import smach
import smach_ros
import math
import os

from com.drive_thru.drive_thru import Drive_thru
from bbauv_msgs.msg import *
from bbauv_msgs.srv import * 

# Dynamic Reconfigure

from dynamic_reconfigure.server import Server
from Vision.cfg import DrivethruConfig

# External libraries
import numpy as np
from rospy.timer import sleep

'''
###################################################################

               TEST MODE TRIGGER
        
###################################################################

'''
isTest = False

'''
###################################################################

               SMACH STATE MACHINE CLASS DECLARATION
        
###################################################################

'''
class Disengage(smach.State):
    client = None
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_complete','phase2_complete', 'complete_outcome', 'aborted'],
                            input_keys=['complete'])
    def execute(self, userdata):
        global locomotionGoal
        global isStart
        global isEnd
        global mission_srv_request
        global movement_client
        global dt
        global isTest
        global r
        global counter
        if userdata.complete:
             isStart = False
             if counter == 2:
                rospy.signal_shutdown("Deactivating Gate Node")
                return 'complete_outcome'
             try:
                 if isTest == False:
                     resp = mission_srv_request(False, True, locomotionGoal)
             except rospy.ServiceException, e:
                 print "Service call failed: %s" % e
            

        while not rospy.is_shutdown():
            if isStart:
                isStart = False
                if counter == 1:
                    return 'phase2_complete'
                elif counter == 0:
                    dt.register()
                    return 'start_complete'
            r.sleep()
        return 'aborted'
    
class Search(smach.State):
    global dt
    global mission_srv_request
   # global r
    def __init__(self):
        smach.State.__init__(self, outcomes=['search_complete', 'aborted', 'mission_abort'])
        
    def execute(self, userdata):
       global r
       global dt
       global isAbort
       global isTest
       while not rospy.is_shutdown():
           if dt.pipe_skeleton_pose != None:
               if dt.pipe_skeleton_pose.detect_pipe:
                   break
           if isAbort:
                return "mission_abort"
           r.sleep()
       if rospy.is_shutdown():
           return 'aborted'
       else:
           if isTest == False:
               try:
                   resp = mission_srv_request(True, False, None)
               except rospy.ServiceException, e:
                   print "Service call failed: %s" % e
           return 'search_complete'

class Centering(smach.State):
    global mission_srv_request
   # global r
    def __init__(self):
        smach.State.__init__(self, outcomes=['centering_complete', 'aborted','identification_failed', 'mission_abort'], output_keys=['center_pos'])
        
    def execute(self, userdata):
        global r
        global dt
        global movement_client
        global locomotionGoal
        global drivethru_params
        global isAbort
        orientation_error = 0
        isOrientationDone = False
        center_complete = False
        locomotionGoal.heading_setpoint = dt.yaw
        while(not center_complete and not rospy.is_shutdown()):
            if isAbort:
                return 'mission_abort'
            if(dt.centroid != None):
                side_error = drivethru_params['centering_y'] * (dt.centroid[0] - dt.cols / 2)
                fwd_error = -drivethru_params['centering_x'] * (dt.centroid[1] - dt.rows / 2)
                ###Conditional Checks for robust identification of target obstacles
                #if dt.find_times < -250:
                #    return "identification_failed"
                if(dt.orientation != 0):
                    if isOrientationDone == False:
                        if(dt.orientation > 90):
                            orientation_error = (dt.yaw + (180 - dt.orientation)) % 360
                        else:
                            orientation_error = (dt.yaw - dt.orientation) % 360
                        rospy.loginfo("selected orient:" + str(dt.orientation) + "ahrs yaw:" + str(dt.yaw) + "final yaw:" + str(orientation_error))
                        isOrientationDone = True
                        locomotionGoal.heading_setpoint = orientation_error
                else:
                    orientation_error = locomotionGoal.heading_setpoint
                if (np.fabs(dt.centroid[0] - dt.cols / 2) < dt.inner_center) and np.fabs(dt.centroid[1] - dt.rows / 2) < dt.inner_center and np.fabs(orientation_error - dt.yaw) < 5 and isOrientationDone :
                    if dt.find_times > 30:
                        movement_client.cancel_all_goals()
                        return "centering_complete"
                    else:
                        return "identification_failed"
                # print "orientation error:" + str(orientation_error) + "isCentering:" + str(dt.isCentering)
                goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=fwd_error, heading_setpoint=orientation_error, depth_setpoint=locomotionGoal.depth_setpoint, sidemove_setpoint=side_error)
                movement_client.send_goal(goal)
                movement_client.wait_for_result(rospy.Duration(2))
            r.sleep()
        if rospy.is_shutdown():
            return 'aborted'
       
class Aiming(smach.State):
    global mission_srv_request
    isLowering = True
    def __init__(self):
        smach.State.__init__(self, outcomes=['aiming_complete', 'aborted', 'mission_abort'], input_keys=['center_pos'])
        
    def execute(self, userdata):
        global r
        global dt
        global movement_client
        global locomotionGoal
        global drivethru_params
        global isAbort
        global isTest
        depth_offset = 0
        while not rospy.is_shutdown():
            if isAbort:
                return "mission_abort"
            x_error = dt.centroid[0] - dt.cols / 2
            y_error = dt.centroid[1] - dt.rows / 2
            
            side_error = drivethru_params['aiming_y'] * (x_error)
            fwd_error = -drivethru_params['aiming_x'] * (y_error)
            
            ''' Area selection criterion for stoppage of lowering'''
            if isTest:
                print dt.max_area
            if dt.max_area > drivethru_params['bin_area']:
                 self.isLowering = False
            if ((np.fabs(dt.centroid[0] - dt.cols / 2) < dt.inner_center and np.fabs(dt.centroid[1] - dt.rows / 2) < dt.inner_center) and dt.max_area > drivethru_params['bin_area'] - 5000):
                locomotionGoal.depth_setpoint = locomotionGoal.depth_setpoint + depth_offset
                rospy.loginfo("Identifying target...")
                return "aiming_complete"
                movement_client.cancel_all_goals()
            if self.isLowering:
                depth_offset = depth_offset + 0.05
            goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=fwd_error,
                                                     heading_setpoint=locomotionGoal.heading_setpoint,
                                                     depth_setpoint=locomotionGoal.depth_setpoint + depth_offset,
                                                     sidemove_setpoint=side_error)
            movement_client.send_goal(goal)
            movement_client.wait_for_result(rospy.Duration(1))
            #rospy.loginfo("isLowering:" + str(self.isLowering))
            r.sleep()
        if rospy.is_shutdown():
            return 'aborted'
        else:
            return 'aiming_complete'

class Grabbing(smach.State):
    Kx = 0.001
    Ky = 0.002
    def __init__(self):
        smach.State.__init__(self, outcomes=['grabbing_complete', "firing_all_complete", 'aborted', 'mission_abort'])
    def fire_grabber(self, grab):
        global mani_pub
        _manipulator = manipulator()
        _manipulator.servo1 = 0
        _manipulator.servo2 = 0
        _manipulator.servo3 = 0
        _manipulator.servo4 = 0
        if grab:
            _manipulator.servo5 = 1
        else:
            _manipulator.servo5 = 0
        _manipulator.servo6 = 0
        _manipulator.servo7 = 0
        mani_pub.publish(_manipulator)
    def execute(self, userdata):
        global dt
        global movement_client
        global locomotionGoal
        global speedtrap_params
        x_error = 0
        y_error = 0
        if isAbort:
            return "mission_abort"
        rospy.loginfo("Pizza Box Target locked. Descending to pick up.")
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=-0.10,
                                             heading_setpoint=locomotionGoal.heading_setpoint,
                                             depth_setpoint=locomotionGoal.depth_setpoint + 1.25,
                                             sidemove_setpoint=-0.03)
        movement_client.send_goal(goal)
        movement_client.wait_for_result(rospy.Duration(10))
        self.fire_grabber(True)
        rospy.loginfo("Fire grabber!")
        rospy.loginfo("Going to sleep. Waiting for Grabber")
        rospy.sleep(rospy.Duration(2))
        return "grabbing_complete"
        if rospy.is_shutdown():
            return 'aborted'
        else:
            return 'firing_complete'

class Manuoevre(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['manuoevre_complete', 'aborted', 'mission_abort'],
                             output_keys=['complete'])
    def execute(self, userdata):
        global r
        global dt
        global movement_client
        global locomotionGoal
        global counter
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0,
                                                     heading_setpoint=locomotionGoal.heading_setpoint,
                                                     depth_setpoint=0,
                                                     sidemove_setpoint=0)
        movement_client.send_goal(goal)
        movement_client.wait_for_result(rospy.Duration(2))
        counter = 1
        rospy.loginfo("Surfacing vehicle!")
        userdata.complete = True
        dt.unregister()
        if rospy.is_shutdown():
            return 'aborted'
        return 'manuoevre_complete'

class Release(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['release_complete', 'aborted', 'mission_abort'],
                             output_keys=['complete'])
    def fire_grabber(self, grab):
        global mani_pub
        _manipulator = manipulator()
        _manipulator.servo1 = 0
        _manipulator.servo2 = 0
        _manipulator.servo3 = 0
        _manipulator.servo4 = 0
        if grab:
            _manipulator.servo5 = 1
        else:
            _manipulator.servo5 = 0
        _manipulator.servo6 = 0
        _manipulator.servo7 = 0
        mani_pub.publish(_manipulator)
        
    def execute(self, userdata):
        global r
        global dt
        global movement_client
        global locomotionGoal
        global counter
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0,
                                                     heading_setpoint=locomotionGoal.heading_setpoint,
                                                     depth_setpoint=2.5,
                                                     sidemove_setpoint=0)
        movement_client.send_goal(goal)
        movement_client.wait_for_result(rospy.Duration(15))
        self.fire_grabber(False)
        counter = 2
        rospy.loginfo("Releasing Grabber.")
        rospy.sleep(rospy.Duration(10))
	userdata.complete = True
        if rospy.is_shutdown():
            return 'aborted'
        return 'release_complete'
'''
###################################################################

                       MAIN PYTHON THREAD
        
###################################################################
'''

def handle_srv(req):
    global isStart
    global isAbort
    global locomotionGoal
    global dt
    
    
    if req.start_request:
        isStart = True
        isAbort = False
        print req.start_ctrl.depth_setpoint
        print req.start_ctrl.heading_setpoint
        # Format for service: start_response, abort_response
        locomotionGoal = req.start_ctrl
    if req.abort_request:
        rospy.loginfo("Drive_thru abort received")
        isAbort = True
        isStart = False
        dt.unregister()
    return mission_to_visionResponse(isStart, isAbort)

# Global Variables
movement_client = None
locomotionGoal = None 
isStart = False
isAbort = False  
isEnd = False
dt = None     
r = None
mani_pub = None
counter = 0
drivethru_params = {'bin_area': 0,'shape_hu':0.0, 'firing_x':10, 'firing_y':0, 'centering_x':0, 'centering_y':0, 'aiming_x':0, 'aiming_y':0}

if __name__ == '__main__':
    rospy.init_node('Drivethru', anonymous=False)
    isTest = rospy.get_param('~testmode',False)
    if isTest:
        isStart = True
    else:
        isStart = False
    r = rospy.Rate(20)
    movement_client = actionlib.SimpleActionClient('LocomotionServer', bbauv_msgs.msg.ControllerAction)
    movement_client.wait_for_server()
    mani_pub = rospy.Publisher("/manipulators", manipulator)
    if isTest:
        locomotionGoal = bbauv_msgs.msg.ControllerGoal()
        locomotionGoal.heading_setpoint = 130
        locomotionGoal.depth_setpoint = 0.6
    else:
        ### Initialize locomotion goal for for mission
        locomotionGoal = bbauv_msgs.msg.ControllerGoal()
        locomotionGoal.heading_setpoint = 0
        locomotionGoal.depth_setpoint = 0.6
    vision_srv = rospy.Service('drivethru_srv', mission_to_vision, handle_srv)
    rospy.loginfo('drivethru_srv initialized!')
    
    dt = Drive_thru(False)
    rospy.loginfo("Drive Thru loaded!")
    # Set up param configuration window
    def drivethruCallback(config, level):
        global dt
        dt.shape_hu = config['shape_hu']
        for param in dt.orange_params:
            dt.orange_params[param] = config['orange_' + param]
        for param in drivethru_params:
            drivethru_params[param] = config[param]
        return config
    srv = Server(DrivethruConfig, drivethruCallback)
   
   # Service Client
    if not isTest:
        rospy.loginfo('waiting for mission_srv...')
        rospy.wait_for_service('mission_srv')
        mission_srv_request = rospy.ServiceProxy('mission_srv', vision_to_mission,headers={'id':'5'})
        rospy.loginfo('connected to mission_srv!')
        
    sm_top = smach.StateMachine(outcomes=['drivethru_complete', 'aborted'])
    # Add overall States to State Machine for Gate Task 
    with sm_top:
        smach.StateMachine.add('DISENGAGED',
                         Disengage(),
                         transitions={'start_complete':'SEARCH', 'complete_outcome':'drivethru_complete', 'aborted':'aborted',
                                      'phase2_complete':'RELEASE'}
                         )
        smach.StateMachine.add('SEARCH',
                         Search(),
                         transitions={'search_complete':'CENTERING', 'aborted':'aborted', 'mission_abort':'DISENGAGED'})
        smach.StateMachine.add('CENTERING',
                         Centering(),
                         transitions={'centering_complete': 'AIMING','identification_failed':'DISENGAGED','aborted':'aborted',
                                      'mission_abort':'DISENGAGED'}
                         )
        smach.StateMachine.add('AIMING',
                         Aiming(),
                         transitions={'aiming_complete': 'GRABBING', 'aborted':'aborted',
                                      'mission_abort':'DISENGAGED'}
                         )
        smach.StateMachine.add('GRABBING',
                         Grabbing(),
                         transitions={'grabbing_complete': 'MANUOEVRE',
                                      'firing_all_complete':'DISENGAGED',
                                      'aborted':'aborted', 'mission_abort':'DISENGAGED'}
                         )
        smach.StateMachine.add('MANUOEVRE',
                         Manuoevre(),
                         transitions={'manuoevre_complete': 'DISENGAGED', 'aborted':'aborted',
                                      'mission_abort':'DISENGAGED'}
                         )
        smach.StateMachine.add('RELEASE',
                         Release(),
                         transitions={'release_complete': 'DISENGAGED', 'aborted':'aborted',
                                      'mission_abort':'DISENGAGED'}
                         )
        
    sis = smach_ros.IntrospectionServer('server', sm_top, '/MISSION/DRIVE_THRU')
    sis.start()
    sm_top.userdata.complete = 0
    # Execute SMACH plan
    outcome = sm_top.execute()
    try:
        rospy.spin()
        # sis.stop()
    except KeyboardInterrupt:
        print "Shutting down"
    pass
