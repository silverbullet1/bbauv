#!/usr/bin/env python2

'''
Created on Apr 25, 2013

@author: gohew
'''
#ROS and System libraries
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

#Dynamic Reconfigure

from dynamic_reconfigure.server import Server
from Vision.cfg import SpeedTrapConfig

#External libraries
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
        smach.State.__init__(self, outcomes=['start_complete','complete_outcome','aborted'],
                            input_keys=['complete'])
    def execute(self,userdata):
        global locomotionGoal
        global isStart
        global isEnd
        global mission_srv_request
        
        if userdata.complete == True:
             isStart = False
             isEnd = True
             try:
                 locomotionGoal.depth_setpoint = 0.6
                 resp = mission_srv_request(False,True,locomotionGoal)
             except rospy.ServiceException, e:
                 print "Service call failed: %s"%e
        while not rospy.is_shutdown():
            if isEnd:
                rospy.signal_shutdown("Deactivating Gate Node")
                return 'complete_outcome'
            if isStart:
                return 'start_complete'
        return 'aborted'
    
class Search(smach.State):
    global st
    global mission_srv_request
   # global r
    def __init__(self):
        smach.State.__init__(self, outcomes=['search_complete','aborted'])
        
    def execute(self,userdata):
       global r
       global st
       while len(st.angleList) == 0 and not rospy.is_shutdown():
           r.sleep()
       if rospy.is_shutdown():
           return 'aborted'
       else:
    
        try:
            resp = mission_srv_request(True,False,None)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return 'search_complete'

class Centering(smach.State):
    global mission_srv_request
   # global r
    Kx = 0.005
    Ky = 0.007
    def __init__(self):
        smach.State.__init__(self, outcomes=['centering_complete','aborted'],output_keys=['center_pos'])
        
    def execute(self,userdata):
        global r
        global st
        global movement_client
        global locomotionGoal
        orientation_error = 0
        isOrientationDone = False
        center_complete = False
        while(not center_complete and not rospy.is_shutdown()):
            if(st.centroidx != 0):
                side_error = self.Ky*(st.centroidx - st.cols/2)
                fwd_error = -self.Kx*(st.centroidy - st.rows/2)
                if(st.orientation != None):
                    if st.isCentering and isOrientationDone == False:
                        if(st.orientation > 90):
                            orientation_error = (st.yaw - (180 - st.orientation)) % 360
                        else:
                            orientation_error = (st.yaw + st.orientation) % 360
                        print st.angleList
                        rospy.loginfo("box_orient:" + str(st.orientation) + "yaw:" + str(st.yaw) +  "final yaw:" + str(orientation_error))
                        isOrientationDone = True
                else:
                    orientation_error = locomotionGoal.heading_setpoint
                if(np.fabs(st.centroidx - st.cols/2) < st.outer_center and np.fabs(st.centroidy - st.rows/2) <st.outer_center and np.fabs(orientation_error - st.yaw) <5):
                    #userdata.center_pos = st.position
                    locomotionGoal.heading_setpoint = orientation_error
                    movement_client.cancel_all_goals()
                    return "centering_complete"
                #print "orientation error:" + str(orientation_error) + "isCentering:" + str(st.isCentering)
                goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=fwd_error,heading_setpoint=orientation_error,depth_setpoint=locomotionGoal.depth_setpoint,sidemove_setpoint=side_error)
                movement_client.send_goal(goal)
                movement_client.wait_for_result(rospy.Duration(2))
                #rospy.loginfo("Centering...")
            r.sleep()
        if rospy.is_shutdown():
            return 'aborted'
        else:
            '''
            try:
                resp = mission_srv_request(True,False,None)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            '''
            return 'search_complete'
       
class Aiming(smach.State):
    global mission_srv_request
    Kx = 0.004
    Ky = 0.006
    isLowering = True
    def __init__(self):
        smach.State.__init__(self, outcomes=['aiming_complete','aborted'],input_keys=['center_pos'])
        
    def execute(self,userdata):
        global r
        global st
        global movement_client
        global locomotionGoal
        depth_offset = 0
        while(len(st.centroidx_list) > 0 and not rospy.is_shutdown()):
            aim_x = np.min(st.centroidx_list, None, None)
            aim_y = np.min(st.centroidy_list, None, None)
            print st.max_area
            side_error = self.Ky*(aim_x - (st.cols/2))
            fwd_error = -self.Kx*(aim_y - st.rows/2)
            if st.max_area > 40000:
                 self.isLowering = False
            if ((np.fabs(aim_x - st.cols/2) <st.outer_center and np.fabs(aim_y - st.rows/2) <st.outer_center) and st.max_area > 38000 ) or (depth_offset + locomotionGoal.depth_setpoint) > 4 :
                locomotionGoal.depth_setpoint = locomotionGoal.depth_setpoint  + depth_offset
                st.isAim = True
                rospy.loginfo("Identifying target...")
                rospy.sleep(rospy.Duration(5))
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
            rospy.loginfo("isLowering:" + str(self.isLowering))
            r.sleep()
        if rospy.is_shutdown():
            return 'aborted'
        else:
            try:
                resp = mission_srv_request(True,False,None)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            return 'aiming_complete'

class Firing(smach.State):
    Kx = 0.0015
    Ky = 0.0030
    def __init__(self):
        smach.State.__init__(self, outcomes=['firing_complete',"firing_all_complete",'aborted'],
                             output_keys=['complete'])
    def fire_dropper(self,left):
        global mani_pub
        _manipulator = manipulator()
        if(left):
            _manipulator.servo1 = 1
            _manipulator.servo2 = 0
        else:
            _manipulator.servo1 = 0
            _manipulator.servo2 = 1
        _manipulator.servo3 = 1
        _manipulator.servo4 = 1
        _manipulator.servo5 = 0
        _manipulator.servo6 = 0
        _manipulator.servo7 = 0
        mani_pub.publish(_manipulator)
    def execute(self,userdata):
        global r
        global st
        global movement_client
        global locomotionGoal
        global count
        x_error = 0
        y_error = 0
        while(len(st.centroidx_list) > 0 and not rospy.is_shutdown()):
            aim_x = np.max(st.centroidx_list, None, None)
            aim_y = np.min(st.centroidy_list, None, None)
            if count == 0:
                x_error = aim_x - (st.cols/2-150)
                y_error = aim_y - st.rows/2
                side_error = self.Ky*x_error
                fwd_error = -self.Kx*y_error
                print "left correct"
            else:
                x_error = aim_x - (st.cols/2+150)
                y_error = aim_y - st.rows/2
                side_error = self.Ky*(x_error)
                fwd_error = -self.Kx*(y_error)
                print "right correct"
            if ((np.fabs(x_error) <st.outer_center and np.fabs(y_error) <st.outer_center) ) :
                print "Fire aim success"
                st.isAim = False
                if count == 0:
                    movement_client.cancel_all_goals()
                    self.fire_dropper(False)
                    rospy.loginfo("Fire left Dropper!")
                    rospy.loginfo("Going to sleep. Waiting for Dropper")
                    rospy.sleep(rospy.Duration(5))
                    count = 1
                    st.counter = 1
                    return "firing_complete"
                if count == 1:
                    movement_client.cancel_all_goals()
                    self.fire_dropper(True)
                    rospy.loginfo("Fire right Dropper!")
                    rospy.loginfo("Going to sleep. Waiting for Dropper")
                    rospy.sleep(rospy.Duration(5))
                    count = 2
                    st.counter = 2
                    userdata.complete = True
                    return "firing_all_complete"
                #rospy.loginfo("Identifying targ...")
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
    Kx = 0.0015
    Ky = 0.0030
    def __init__(self):
        smach.State.__init__(self, outcomes=['manuoevre_complete','aborted'])
    def execute(self,userdata):
        global r
        global st
        global movement_client
        global locomotionGoal
        global count
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0,
                                                     heading_setpoint=locomotionGoal.heading_setpoint,
                                                     depth_setpoint=locomotionGoal.depth_setpoint,
                                                     sidemove_setpoint=1.5)
        movement_client.send_goal(goal)
        movement_client.wait_for_result(rospy.Duration(10))
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
    rospy.loginfo("Speed Trap service handled.")
    if req.start_request:
        rospy.loginfo("isStart true.")
        isStart = True
        # Format for service: start_response, abort_response
        locomotionGoal = req.start_ctrl
    if req.abort_request:
        isAbort = True
    return mission_to_visionResponse(isStart,isAbort)

#Global Variables

movement_client = None
locomotionGoal = None 
isStart = False
isAbort = False  
isEnd = False
count = 0
st = None     
r = None
mani_pub = None
if __name__ == '__main__':
    rospy.init_node('SpeedTrap', anonymous=False)
    r = rospy.Rate(20)
    movement_client = actionlib.SimpleActionClient('LocomotionServer', bbauv_msgs.msg.ControllerAction)
    movement_client.wait_for_server()
    mani_pub = rospy.Publisher("/manipulators",manipulator)
    locomotionGoal = bbauv_msgs.msg.ControllerGoal()
    locomotionGoal.heading_setpoint = 130
    locomotionGoal.depth_setpoint = 0.6
    vision_srv = rospy.Service('speedtrap_srv', mission_to_vision, handle_srv)
    rospy.loginfo('speedtrap_srv initialized!')
    
    #Service Client
    rospy.loginfo('waiting for mission_srv...')
    rospy.wait_for_service('mission_srv')
    mission_srv_request = rospy.ServiceProxy('mission_srv', vision_to_mission)
    rospy.loginfo('connected to mission_srv!')
    st = SpeedTrap()
    rospy.loginfo("SpeedTrap loaded!")
    # Set up param configuration window
    def configCallback(config, level):
        for param in st.yellow_params:
            st.yellow_params[param] = config['yellow_' + param]
        for param in st.red_params:
            st.red_params[param] = config['red_'+param]
        return config
    srv = Server(SpeedTrapConfig, configCallback)
   
    sm_top = smach.StateMachine(outcomes=['speedtrap_complete','aborted'])
    #Add overall States to State Machine for Gate Task 
    with sm_top:
        smach.StateMachine.add('DISENGAGED',
                         Disengage(),
                         transitions={'start_complete':'SEARCH','complete_outcome':'speedtrap_complete','aborted':'aborted'}
                         )
        smach.StateMachine.add('SEARCH',
                         Search(),
                         transitions={'search_complete':'CENTERING','aborted':'aborted'})
        smach.StateMachine.add('CENTERING',
                         Centering(),
                         transitions={'centering_complete': 'AIMING','aborted':'aborted'}
                         )
        smach.StateMachine.add('AIMING',
                         Aiming(),
                         transitions={'aiming_complete': 'FIRING','aborted':'aborted'}
                         )
        smach.StateMachine.add('FIRING',
                         Firing(),
                         transitions={'firing_complete': 'MANUOEVRE',
                                      'firing_all_complete':'DISENGAGED',
                                      'aborted':'aborted'}
                         )
        smach.StateMachine.add('MANUOEVRE',
                         Manuoevre(),
                         transitions={'manuoevre_complete': 'FIRING','aborted':'aborted'}
                         )

    sis = smach_ros.IntrospectionServer('server',sm_top,'/MISSION/SPEEDTRAP')
    sis.start()
    sm_top.userdata.complete = False
    # Execute SMACH plan
    outcome = sm_top.execute()
    try:
        rospy.spin()
        #sis.stop()
    except KeyboardInterrupt:
        print "Shutting down"
    pass