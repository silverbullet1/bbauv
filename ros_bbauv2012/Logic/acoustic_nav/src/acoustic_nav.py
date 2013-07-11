#! /usr/bin/env python

import roslib; roslib.load_manifest('Vision')
import rospy
import actionlib

import smach
import smach_ros

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from geometry_msgs.msg import *

import math
import numpy as np

### State Declaration ###
#1: Disengage - start/stop control of the node
class Disengage(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_complete', 'complete_outcome', 'aborted'],
                            input_keys=['complete'])
    def execute(self,userdata):
        #maintain heading after finishing 
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

        while not rospy.is_shutdown():
            if isEnd:
                rospy.signal_shutdown("Deactivating Acoustic Node")
                return 'complete_outcome'
            if isStart:
                acoustic_nav.register()
                print "starting Acoustic node"
                return 'start_complete'
        return 'aborted'
#2: CorrectHeading - use far field equation
class CorrectHeading(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['correct_heading_completed','aborted'])
    def execute(self,userdata):
        r=rospy.Rate(0.5) #run at 2Hz
        rel_angle=360 #fake angle to start the loop. comment: better use do... while (...)
        while (np.fabs(rel_angle)>10):
            far_field()
            rel_yaw = (math.atan2(y,x)*180/math.pi) 
            if (rel_yaw != old_yaw):
                print 'x={0:.3f},y={1:.3f},z={2:.3f},yaw={3:.3f},rel_yaw={4:.3f}'.format(x,y,z,yaw,rel_yaw)
                if (np.fabs(rel_yaw) < 80):
                    new_heading = (yaw + rel_yaw)%360
                    print 'correcting heading to {0:.3f}'.format(new_heading) 
                    goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0,heading_setpoint=new_heading,depth_setpoint=1,sidemove_setpoint=0)
                    client.send_goal(goal)
                    client.wait_for_result(rospy.Duration(30))
                else:
                    print 'suspect wrong angle! rel_yaw = {0}'.format(rel_yaw)
                old_yaw=rel_yaw
            else:
                print 'waiting for new TDOA'
            r.sleep() 
        return 'correct_heading_completed'
        
#3: SearchAhead - keep current heading, search until certain condition to use near field equation
class SearchAhead(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['search_finished','aborted'])
    #def execute(self,userdata):
        #while z<? (to determine how close)
            #update goal to move forward
#4: DriveThru - use near file equation and find exact position of the pinger
class DriveThru(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['drive_finished','aborted'])
    #def execute(self,userdata):
        #call nearfield
        #send final forward & sidemove goal
        #notify finish driving, go back to disengage state

def TDOA_sub_callback(msg):
    d10=msg.linear.x/1000000*speedOfSound
    d20=msg.linear.y/1000000*speedOfSound
    d30=msg.linear.z/1000000*speedOfSound

def far_field():
    global x,y,z
    global x_1,x_2,x_3,y_1,y_2,y_3,z_1,z_2,z_3
    global d10,d20,d30

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

def get_heading(data):
    global yaw
    yaw=data.yaw

################### MAIN #####################
#States variabes
isStart = False
isEnd = False
isAbort = False

#Current state of vehicle:
#from Compass, DVL
yaw =0 
altitude =0
#heading from last calculation
old_yaw = 0
#from hydrophones (TDOA)
d10=0
d20=0
d30=0

#Triangulation variables
#hydrophones position
x_1=0
y_1=-0.07
z_1 = 0.07
x_2=0
y_2=0.07
z_2 = 0.07
x_3=0.1
y_3=0
z_3 =0
#const
speedOfSound = 1484
#result (direction/position)
x=0
y=0
z=0

if __name__ == '__main__':
    try:
        rospy.init_node('acoustic_navigation')
        client = actionlib.SimpleActionClient('LocomotionServer', bbauv_msgs.msg.ControllerAction)
        client.wait_for_server()

        rospy.Subscriber("/hydrophone/time_diff",Twist,TDOA_sub_callback)
        rospy.Subscriber("/euler",compass_data,get_heading)
        r=rospy.Rate(0.5) #run at 2Hz

        while not rospy.is_shutdown():
            far_field()
            rel_yaw = (math.atan2(y,x)*180/math.pi) 
            if (rel_yaw != old_yaw):
                print 'x={0:.3f},y={1:.3f},z={2:.3f},yaw={3:.3f},rel_yaw={4:.3f}'.format(x,y,z,yaw,rel_yaw)
                if (np.fabs(rel_yaw) < 80):
                    new_heading = (yaw + rel_yaw)%360
                    if (np.fabs(rel_yaw) >	10):
                        print 'correcting heading to {0:.3f}'.format(new_heading) 
                        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0,heading_setpoint=new_heading,depth_setpoint=1,sidemove_setpoint=0)
                    else:
                        print 'moving forward'
                        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=5,heading_setpoint=yaw,depth_setpoint=1,sidemove_setpoint=0)
                    client.send_goal(goal)
                    client.wait_for_result(rospy.Duration(30))
                else:
                    print 'suspect wrong angle! rel_yaw = {0}'.format(rel_yaw)
                old_yaw=rel_yaw
            else:
                print 'waiting for new TDOA'
            r.sleep() 

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
