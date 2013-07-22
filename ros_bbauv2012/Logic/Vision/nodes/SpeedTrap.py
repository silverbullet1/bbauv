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

from com.speedtrap.speedtrap import SpeedTrap
from bbauv_msgs.msg import *
from bbauv_msgs.srv import * 

# Dynamic Reconfigure

from dynamic_reconfigure.server import Server
from Vision.cfg import SpeedTrapConfig

# External libraries
import numpy as np
from rospy.timer import sleep

'''
###################################################################

               SMACH STATE MACHINE CLASS DECLARATION
        
###################################################################

'''
class Disengage(smach.State):
    client = None
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_complete', 'complete_outcome', 'aborted'],
                            input_keys=['complete'])
    def execute(self, userdata):
        global locomotionGoal
        global isStart
        global isEnd
        global mission_srv_request
        global movement_client
        global st
        global isTest
        global r
        if userdata.complete == True:
             isStart = False
             isEnd = True
             locomotionGoal.depth_setpoint = 0.6
             goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0,
                                                     heading_setpoint=locomotionGoal.heading_setpoint,
                                                     depth_setpoint=locomotionGoal.depth_setpoint,
                                                     sidemove_setpoint=0)
             movement_client.send_goal(goal)
             movement_client.wait_for_result(rospy.Duration(15))     
             try:
                 if isTest == False:
                     resp = mission_srv_request(False, True, locomotionGoal)
                     rospy.loginfo("SpeedTrap Task completed")
             except rospy.ServiceException, e:
                 print "Service call failed: %s" % e
        while not rospy.is_shutdown():
            if isEnd:
                rospy.signal_shutdown("Deactivating Gate Node")
                return 'complete_outcome'
            if isStart:
                isStart = False
                st.register()
                rospy.loginfo("Registering Topics.")
                return 'start_complete'
            r.sleep()
        return 'aborted'
    
class Search(smach.State):
    global st
    global mission_srv_request
   # global r
    def __init__(self):
        smach.State.__init__(self, outcomes=['search_complete', 'aborted', 'mission_abort'])
        
    def execute(self, userdata):
       global r
       global st
       global isAbort
       global isTest
       while len(st.angleList) == 0 and not rospy.is_shutdown():
           if isAbort:
               rospy.loginfo("Aborted by Mission_planner.")
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
    def __init__(self):
        smach.State.__init__(self, outcomes=['centering_complete', 'aborted', 'mission_abort'], output_keys=['center_pos'])
        
        
    def execute(self, userdata):
        global r
        global st
        global movement_client
        global locomotionGoal
        global isAbort
        orientation_error = 0
        isOrientationDone = False
        center_complete = False
        while(not center_complete and not rospy.is_shutdown()):
            if isAbort:
                rospy.loginfo("Aborted by Mission_planner.")
                return 'mission_abort'
            if(st.centroidx != 0):
                side_error = speedtrap_params['centering_y'] * (st.centroidx - st.cols / 2)
                fwd_error = -speedtrap_params['centering_x'] * (st.centroidy - st.rows / 2)
                if(st.orientation != None):
                    if st.isCentering and isOrientationDone == False:
                        if(st.orientation > 90):
                            orientation_error = (st.yaw - (180 - st.orientation)) % 360
                        else:
                            orientation_error = (st.yaw + st.orientation) % 360
                        print st.angleList
                        rospy.loginfo("selected orient:" + str(st.orientation) + "ahrs yaw:" + str(st.yaw) + "final yaw:" + str(orientation_error))
                        isOrientationDone = True
                else:
                    orientation_error = locomotionGoal.heading_setpoint
                if (np.fabs(st.centroidx - st.cols / 2) < st.outer_center) and np.fabs(st.centroidy - st.rows / 2) < st.outer_center and np.fabs(orientation_error - st.yaw) < 5 and isOrientationDone :
                    # userdata.center_pos = st.position
                    locomotionGoal.heading_setpoint = orientation_error
                    movement_client.cancel_all_goals()
                    return "centering_complete"
                # print "orientation error:" + str(orientation_error) + "isCentering:" + str(st.isCentering)
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
        global st
        global movement_client
        global locomotionGoal
        global speedtrap_params
        global isAbort
        global count
        depth_offset = 0
        while(len(st.centroidx_list) > 0 and not rospy.is_shutdown()):
            if isAbort:
                rospy.loginfo("Aborted by Mission_planner.")
                return "mission_abort"
            '''Selection Algorithm for the top left and top right speed trap bins'''
            final_coord = None
            coord_min_y = np.argmin(st.centroidy_list, None)
            for i in range(0,len(st.centroidy_list)):
                if (np.fabs(st.centroidy_list[i] - st.centroidy_list[coord_min_y]) < 50):
                    if count == 0:
                        #Select Top Left
                        if st.centroidx_list[i] < st.centroidx_list[coord_min_y]:
                            final_coord = i
                    elif count == 1:
                        #Select Top Right
                        if st.centroidx_list[i] > st.centroidx_list[coord_min_y]:
                            final_coord = i
            if final_coord == None:
                #rospy.loginfo("coord_min select")
                aim_x = st.centroidx_list[coord_min_y]
                aim_y = st.centroidy_list[coord_min_y]
            else:                
                #rospy.loginfo("final_coord select")
                aim_x = st.centroidx_list[final_coord]
                aim_y = st.centroidy_list[final_coord]
                
            st.aim_point = (int(aim_x),int(aim_y))
            x_error = aim_x - st.cols / 2
            y_error = aim_y - st.rows / 2
            
            side_error = speedtrap_params['aiming_y'] * (x_error)
            fwd_error = -speedtrap_params['aiming_x'] * (y_error)
            
            ''' Area selection criterion for stoppage of lowering'''
            
            print st.max_area
            if st.max_area > speedtrap_params['bin_area']:
                 self.isLowering = False
            if ((np.fabs(aim_x - st.cols / 2) < st.outer_center and np.fabs(aim_y - st.rows / 2) < st.outer_center) and st.max_area > speedtrap_params['bin_area'] - 5000) or (depth_offset + locomotionGoal.depth_setpoint) > 4 :
                locomotionGoal.depth_setpoint = locomotionGoal.depth_setpoint + depth_offset
                st.isAim = True
                rospy.loginfo("Identifying target...")
                rospy.sleep(rospy.Duration(4))
                rospy.loginfo("Target identified") 
                return "aiming_complete"
            if self.isLowering:
                depth_offset = depth_offset + 0.15
            goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=fwd_error,
                                                     heading_setpoint=locomotionGoal.heading_setpoint,
                                                     depth_setpoint=locomotionGoal.depth_setpoint + depth_offset,
                                                     sidemove_setpoint=side_error)
            movement_client.send_goal(goal)
            movement_client.wait_for_result(rospy.Duration(2))
            #rospy.loginfo("isLowering:" + str(self.isLowering))
            r.sleep()
        if rospy.is_shutdown():
            return 'aborted'
        else:
            return 'aiming_complete'

class Firing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['firing_complete', "firing_all_complete", 'aborted', 'mission_abort'],
                             output_keys=['complete'])
    def fire_dropper(self, left):
        global mani_pub
        _manipulator = manipulator()
        if(left):
            _manipulator.servo1 = 1
            _manipulator.servo2 = 0
        else:
            _manipulator.servo1 = 0
            _manipulator.servo2 = 1
        _manipulator.servo3 = 0
        _manipulator.servo4 = 0
        _manipulator.servo5 = 0
        _manipulator.servo6 = 0
        _manipulator.servo7 = 0
        mani_pub.publish(_manipulator)
        mani_pub.publish(_manipulator)
        rospy.loginfo("Going to sleep. Waiting for Dropper")
        rospy.sleep(rospy.Duration(3))
        if(left):
            _manipulator.servo1 = 0
            _manipulator.servo2 = 0
        else:
            _manipulator.servo1 = 0
            _manipulator.servo2 = 0
        _manipulator.servo3 = 0
        _manipulator.servo4 = 0
        _manipulator.servo5 = 0
        _manipulator.servo6 = 0
        _manipulator.servo7 = 0
        mani_pub.publish(_manipulator)
        mani_pub.publish(_manipulator)
        
    def execute(self, userdata):
        global r
        global st
        global movement_client
        global locomotionGoal
        global count
        global speedtrap_params
        x_error = 0
        y_error = 0
        while(len(st.centroidx_list) > 0 and not rospy.is_shutdown()):
            if isAbort:
                rospy.loginfo("Firing Mission timeout. Releasing ammo now!")
                self.fire_dropper(True)
                self.fire_dropper(False)
                return "mission_abort"
            
            '''Selection Algorithm for the top left and top right speed trap bins'''
            final_coord = None
            coord_min_y = np.argmin(st.centroidy_list, None)
            for i in range(0,len(st.centroidy_list)):
                if (np.fabs(st.centroidy_list[i] - st.centroidy_list[coord_min_y]) < 50):
                    if count == 0:
                        #Select Top Left
                        if st.centroidx_list[i] < st.centroidx_list[coord_min_y]:
                            final_coord = i
                    elif count == 1:
                        #Select Top Right
                        if st.centroidx_list[i] > st.centroidx_list[coord_min_y]:
                            final_coord = i
            if final_coord == None:
                aim_x = st.centroidx_list[coord_min_y]
                aim_y = st.centroidy_list[coord_min_y]
            else:                
                aim_x = st.centroidx_list[final_coord]
                aim_y = st.centroidy_list[final_coord]
            if count == 0:
                st.aim_point = (int(aim_x -100),int(aim_y))
                x_error = aim_x - st.cols / 2 + 150 
                y_error = aim_y - st.rows / 2
            elif count == 1:
                st.aim_point = (int(aim_x + 100),int(aim_y))
                x_error = aim_x - st.cols / 2 - 150
                y_error = aim_y - st.rows / 2
            side_error = speedtrap_params['firing_y'] * (x_error)
            fwd_error = -speedtrap_params['firing_x'] * (y_error)
            if ((np.fabs(x_error) < st.outer_center and np.fabs(y_error) < st.outer_center)) :
                print "Fire aim success"
                st.isAim = False
                if count == 0:
                    movement_client.cancel_all_goals()
                    self.fire_dropper(True)
                    rospy.loginfo("Fire left Dropper!")
                    rospy.loginfo("Going to sleep. Waiting for Dropper")
                    rospy.sleep(rospy.Duration(3))
                    count = 1
                    st.counter = 1
                    return "firing_complete"
                if count == 1:
                    movement_client.cancel_all_goals()
                    self.fire_dropper(False)
                    rospy.loginfo("Fire right Dropper!")
                    count = 2
                    st.counter = 2
                    userdata.complete = True
                    return "firing_all_complete"
                # rospy.loginfo("Identifying targ...")
            goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=fwd_error,
                                                     heading_setpoint=locomotionGoal.heading_setpoint,
                                                     depth_setpoint=locomotionGoal.depth_setpoint,
                                                     sidemove_setpoint=side_error)
            movement_client.send_goal(goal)
            movement_client.wait_for_result(rospy.Duration(1))
            r.sleep()
        if rospy.is_shutdown():
            return 'aborted'
        else:
            return 'firing_complete'

class Manuoevre(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['manuoevre_complete', 'aborted', 'mission_abort'])
    def execute(self, userdata):
        global r
        global st
        global movement_client
        global locomotionGoal
        global count
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0,
                                                     heading_setpoint=locomotionGoal.heading_setpoint,
                                                     depth_setpoint=locomotionGoal.depth_setpoint,
                                                     sidemove_setpoint=0.7)
        movement_client.send_goal(goal)
        movement_client.wait_for_result(rospy.Duration(30))
        print st.centroidx_list
        if st.centroidx == 0:
            rospy.loginfo("Bin not found reversing left")
            goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0,
                                                     heading_setpoint=locomotionGoal.heading_setpoint,
                                                     depth_setpoint=locomotionGoal.depth_setpoint,
                                                     sidemove_setpoint=-1.8)
            movement_client.send_goal(goal)
            movement_client.wait_for_result(rospy.Duration(30))
	rospy.loginfo("sway movement to second bin complete!")
        return "manuoevre_complete"
        if rospy.is_shutdown():
            return 'aborted'
        else:
            return 'firing_complete'


'''
###################################################################

                       MAIN PYTHON THREAD
        
###################################################################
'''

def handle_srv(req):
    global isStart
    global isAbort
    global locomotionGoal
    global st
    rospy.loginfo("Speed Trap service handled.")
    if req.start_request:
        rospy.loginfo("isStart true.")
        isStart = True
        isAbort = False
        # Format for service: start_response, abort_response
        locomotionGoal = req.start_ctrl
    if req.abort_request:
        rospy.loginfo("SpeedTrap abort received")
        isAbort = True
        isStart = False
        st.unregister()
    return mission_to_visionResponse(isStart, isAbort)

# Global Variables
isTest = False
movement_client = None
locomotionGoal = None 
isStart = False
isAbort = False  
isEnd = False
count = 0
st = None     
r = None
mani_pub = None
speedtrap_params = {'bin_area': 0, 'firing_x':10, 'firing_y':0, 'centering_x':0, 'centering_y':0, 'aiming_x':0, 'aiming_y':0}

if __name__ == '__main__':
    rospy.init_node('SpeedTrap', anonymous=False)
    r = rospy.Rate(20)
    st = SpeedTrap(False)
    rospy.loginfo("SpeedTrap loaded!")
    
    # Set up param configuration window
    def speedtrapCallback(config, level):
        for param in st.yellow_params:
            st.yellow_params[param] = config['yellow_' + param]
        for param in st.red_params:
            st.red_params[param] = config['red_' + param]
        for param in speedtrap_params:
            speedtrap_params[param] = config[param]
        isTest = config['test_mode']
        return config
    srv = Server(SpeedTrapConfig, speedtrapCallback)
    
    if isTest:
        isStart = True
    movement_client = actionlib.SimpleActionClient('LocomotionServer', bbauv_msgs.msg.ControllerAction)
    movement_client.wait_for_server()
    mani_pub = rospy.Publisher("/manipulators", manipulator)
    if isTest:
        locomotionGoal = bbauv_msgs.msg.ControllerGoal()
        locomotionGoal.heading_setpoint = 130
        locomotionGoal.depth_setpoint = 0.6
    vision_srv = rospy.Service('speedtrap_srv', mission_to_vision, handle_srv)
    rospy.loginfo('speedtrap_srv initialized!')
    
    # Service Client
    if not isTest:
        rospy.loginfo('waiting for mission_srv...')
        rospy.wait_for_service('mission_srv')
        mission_srv_request = rospy.ServiceProxy('mission_srv', vision_to_mission,headers={'id':'3'})
        rospy.loginfo('connected to mission_srv!')
   
    sm_top = smach.StateMachine(outcomes=['speedtrap_complete', 'aborted'])
    # Add overall States to State Machine for Gate Task 
    with sm_top:
        smach.StateMachine.add('DISENGAGED',
                         Disengage(),
                         transitions={'start_complete':'SEARCH', 'complete_outcome':'speedtrap_complete', 'aborted':'aborted'}
                         )
        smach.StateMachine.add('SEARCH',
                         Search(),
                         transitions={'search_complete':'CENTERING', 'aborted':'aborted', 'mission_abort':'DISENGAGED'})
        smach.StateMachine.add('CENTERING',
                         Centering(),
                         transitions={'centering_complete': 'AIMING', 'aborted':'aborted',
                                      'mission_abort':'DISENGAGED'}
                         )
        smach.StateMachine.add('AIMING',
                         Aiming(),
                         transitions={'aiming_complete': 'FIRING', 'aborted':'aborted',
                                      'mission_abort':'DISENGAGED'}
                         )
        smach.StateMachine.add('FIRING',
                         Firing(),
                         transitions={'firing_complete': 'MANUOEVRE',
                                      'firing_all_complete':'DISENGAGED',
                                      'aborted':'aborted', 'mission_abort':'DISENGAGED'}
                         )
        smach.StateMachine.add('MANUOEVRE',
                         Manuoevre(),
                         transitions={'manuoevre_complete': 'FIRING', 'aborted':'aborted',
                                      'mission_abort':'DISENGAGED'}
                         )

    sis = smach_ros.IntrospectionServer('server', sm_top, '/MISSION/SPEEDTRAP')
    sis.start()
    sm_top.userdata.complete = False
    # Execute SMACH plan
    outcome = sm_top.execute()
    try:
        rospy.spin()
        # sis.stop()
    except KeyboardInterrupt:
        print "Shutting down"
    pass
