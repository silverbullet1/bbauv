#! /usr/bin/env python

import roslib; roslib.load_manifest('Vision')
import rospy
import actionlib

import smach
import smach_ros

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

import math
import numpy as np

### State Declaration ###
#1: Disengage - start/stop control of the node
class Disengage(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['started', 'completed', 'aborted'])
    def execute(self,userdata):      
        while not rospy.is_shutdown():
            if isEnd:
                rospy.signal_shutdown("Deactivating Acoustic Node")
                return 'completed'
            if isStart:
                print "#1 Starting Acoustic node"
                return 'started'
        return 'aborted'
#2: CorrectHeading - use far field equation
class CorrectHeading(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed','aborted'])
    def execute(self,userdata):
        print '#2 Correcting heading:'
        far_field()
        rel_yaw = (math.atan2(y,x)*180/math.pi) 
        old_rel_yaw=0
        while (np.fabs(rel_yaw)>10) and not rospy.is_shutdown():
            if (rel_yaw != old_rel_yaw):
                print 'x={0:.3f},y={1:.3f},z={2:.3f},yaw={3:.3f},rel_yaw={4:.3f}'.format(x,y,z,yaw,rel_yaw)
                if (np.fabs(rel_yaw) < 130):
                    new_heading = (yaw + rel_yaw)%360
                    print 'correcting heading to {0:.3f}'.format(new_heading) 
                    goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0,heading_setpoint=new_heading,depth_setpoint=0.3,sidemove_setpoint=0)
                    print '####'
                    print goal
                    print '####'
                    movement_client.send_goal(goal)
                    movement_client.wait_for_result(rospy.Duration(30))
                else:
                    print 'suspect wrong angle! rel_yaw = {0}'.format(rel_yaw)
                old_rel_yaw=rel_yaw
            else:
                print 'waiting for new TDOA'
            rospy.sleep(3)
            far_field()
            rel_yaw = (math.atan2(y,x)*180/math.pi) 
            
        #maintain current heading, depth 
        return 'completed'
        
#3: SearchAhead - keep current heading, search until certain condition to use near field equation
class SearchAhead(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed','aborted'])
    def execute(self,userdata):
        print '#3 moving forward'
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=3.5,heading_setpoint=yaw,depth_setpoint=0.3,sidemove_setpoint=0)
        print '####'
        print goal
        print '####'
        movement_client.send_goal(goal)
        movement_client.wait_for_result()
        print 'finished moving forward 3m'
        return 'completed'
        #while z<? (to determine how close)
            #update goal to move forward
#4: DriveThru - use near file equation and find exact position of the pinger
class DriveThru(smach.State):
    global isStart,isEnd
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed','aborted'])
    def execute(self,userdata):
        print '#4 Moving to exact position using near field'
        near_field()
        rel_yaw = (math.atan2(y,x)*180/math.pi)
        print 'x={0:.3f},y={1:.3f},z={2:.3f},yaw={3:.3f},rel_yaw={4:.3f}'.format(x,y,z,yaw,rel_yaw)
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=x,heading_setpoint=yaw,depth_setpoint=0.3,sidemove_setpoint=y)
        print '####'
        print goal
        print '####'
        movement_client.send_goal(goal)
        movement_client.wait_for_result(rospy.Duration(30))
        
        print 'MISSION DONE SURFACING...'
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0,heading_setpoint=yaw,depth_setpoint=0,sidemove_setpoint=0)
        print '####'
        print goal
        print '####'
        movement_client.send_goal(goal)
        movement_client.wait_for_result(rospy.Duration(30))
        
        rospy.signal_shutdown("Deactivating Acoustic Node")

        return 'completed'
        #call nearfield
        #send final forward & sidemove goal
        #notify finish driving, go back to disengage state

def TDOA_sub_callback(msg):
    global d10,d20,d30
    d10=msg.linear.x/1000000*speedOfSound
    d20=msg.linear.y/1000000*speedOfSound
    d30=msg.linear.z/1000000*speedOfSound

def far_field():
    global x,y,z

    detA = x_1 * (y_2 * z_3 - y_3 * z_2)
    detA -= y_1 * (x_2 * z_3 - x_3 * z_2)
    detA += z_1 * (x_2 * y_3 - x_3 * y_2)

    x  = d10 * (y_2 * z_3 - y_3 * z_2)
    x -= d20 * (y_1 * z_3 - y_3 * z_1)
    x += d30 * (y_1 * z_2 - y_2 * z_1)

    y = -d10 * (x_2 * z_3 - x_3 * z_2)
    y += d20 * (x_1 * z_3 - x_3 * z_1)
    y -= d30 * (x_1 * z_2 - x_2 * z_1)

    z  = d10 * (x_2 * y_3 - x_3 * y_2)
    z -= d20 * (x_1 * y_3 - x_3 * y_1)
    z += d30 * (x_1 * y_2 - x_2 * y_1)

    x = x / detA
    y = y / detA
    z = z / detA

    normalize = math.sqrt(x*x+y*y+z*z)
    if (normalize != 0):
        x = x / normalize
        y = y / normalize
        z = z / normalize

def near_field():
    global x,y,z

    z =1.61 #altitude
    
    if (d10!=0 and d20!=0 and d30!=0):
        A2 = 2 * (x_2 / d20 - x_1 / d10)
        B2 = 2 * (y_2 / d20 - y_1 / d10)
        A3 = 2 * (x_3 / d30 - x_1 / d10)
        B3 = 2 * (y_3 / d30 - y_1 / d10)
        
        C2 = z * 2 * (z_2/d20 - z_1/d10)
        C2 += d20 - d10
        C2 += -(x_2 * x_2 + y_2 * y_2 + z_2 * z_2)/d20 + (x_1 * x_1 + y_1 * y_1 + z_1 * z_1)/d10
        C2 = -C2
        C3 = z * 2 * (z_3/d30 - z_1/d10)
        C3 += d30 - d10 
        C3 += -(x_3 * x_3 + y_3 * y_3 + z_3 * z_3)/d30 + (x_1 * x_1 + y_1 * y_1 + z_1 * z_1)/d10
        C3 = -C3

        if (A2*B3 == A3*B2):
            x = 0
            y = 0
        else:
            x = (C2 * B3 - C3 * B2) / (A2 * B3 - A3 * B2)
            y = (C3 * A2 - C2 * A3) / (B3 * A2 - B2 * A3)

def get_heading(msg):
    global yaw
    yaw=msg.yaw

def get_altitude(msg):
    global altitude
    altitude=msg.data

def handle_srv(req):
    global isStart
    global isAbort
    global locomotionGoal
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
    return mission_to_visionResponse(isStart, isAbort)

################### MAIN #####################
#States variabes
isStart = False
isEnd = False
isAbort = False
isTest = True
movement_client = None
locomotionGoal = None 

#Current state of vehicle:
#from Compass, DVL
yaw =0 
altitude =0
#heading from last calculation
old_yaw = 0
#from hydrophones (TDOA)
d10,d20,d30=0,0,0
#Triangulation consts
speedOfSound = 1484
x_1, y_1, z_1 = 0  ,-0.07, 0.07 #left hydrophone
x_2, y_2, z_2 = 0  , 0.07, 0.07 #right hydrophone
x_3, y_3, z_3 = 0.1, 0   , 0    #top hydrophone
#result (direction/position)
x, y ,z =0, 0, 0

if __name__ == '__main__':
    rospy.init_node('acoustic_navigation')
    loop_rate=rospy.Rate(0.5)

    rospy.loginfo("AcousticNavigation activated!")
    rospy.wait_for_service('set_controller_srv')
    set_controller_request=rospy.ServiceProxy('set_controller_srv',set_controller)
    set_controller_request(True,True,True,True,False,False,False)
     
    #Setup movement client
    movement_client = actionlib.SimpleActionClient('LocomotionServer', bbauv_msgs.msg.ControllerAction)
    print 'waiting for action server'
    movement_client.wait_for_server()
    print 'finished waiting'
    #Setup acoustic service 
    #pls rename mission-to-vision to mission-to-task!!!
    #acoustic_srv = rospy.service('acoustic_srv',mission_to_vision,handle_srv)

    if isTest:
        isStart= True
    else:
        rospy.loginfo('waiting for mission_srv...')
        rospy.wait_for_service('mission_srv')
        mission_srv_request = rospy.ServiceProxy('mission_srv', vision_to_mission)
        rospy.loginfo('connected to mission_srv!')

    #Setup Subscribers
    rospy.Subscriber("/hydrophone/time_diff",Twist,TDOA_sub_callback)
    rospy.Subscriber("/euler",compass_data,get_heading)
    rospy.Subscriber("/altitude",Float32,get_altitude)

    #State Machines
    sm_top = smach.StateMachine(outcomes=['Task_completed','Aborted'])
    with sm_top:
        smach.StateMachine.add('DISENGAGE',Disengage(),
            transitions ={'started':'CORRECTHEADING','completed':'Task_completed','aborted':'Aborted'})
        smach.StateMachine.add('CORRECTHEADING',CorrectHeading(),
            transitions ={'completed':'SEARCHAHEAD','aborted':'Aborted'})
        smach.StateMachine.add('SEARCHAHEAD',SearchAhead(),
            transitions ={'completed':'DRIVETHRU','aborted':'Aborted'})
        smach.StateMachine.add('DRIVETHRU',DriveThru(),
            transitions ={'completed':'Task_completed','aborted':'Aborted'})

    sis = smach_ros.IntrospectionServer('server', sm_top, '/MISSION/ACOUSTICNAVIGATION')
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
#-------back up code-----
    # while not rospy.is_shutdown():
    #     far_field()
    #     rel_yaw = (math.atan2(y,x)*180/math.pi) 
    #     if (rel_yaw != old_yaw):
    #         print 'x={0:.3f},y={1:.3f},z={2:.3f},yaw={3:.3f},rel_yaw={4:.3f}'.format(x,y,z,yaw,rel_yaw)
    #         if (np.fabs(rel_yaw) < 80):
    #             new_heading = (yaw + rel_yaw)%360
    #             if (np.fabs(rel_yaw) >	10):
    #                 print 'correcting heading to {0:.3f}'.format(new_heading) 
    #                 goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0,heading_setpoint=new_heading,depth_setpoint=1,sidemove_setpoint=0)
    #             else:
    #                 print 'moving forward'
    #                 goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=5,heading_setpoint=yaw,depth_setpoint=1,sidemove_setpoint=0)
    #             client.send_goal(goal)
    #             client.wait_for_result(rospy.Duration(30))
    #         else:
    #             print 'suspect wrong angle! rel_yaw = {0}'.format(rel_yaw)
    #         old_yaw=rel_yaw
    #     else:
    #         print 'waiting for new TDOA'
    #     r.sleep() 

