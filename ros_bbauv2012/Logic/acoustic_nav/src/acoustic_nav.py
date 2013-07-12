#! /usr/bin/env python

import roslib; roslib.load_manifest('Vision')
import rospy
import actionlib

import smach
import smach_ros

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from Queue import Queue

import math
import numpy as np

tdoa_queue = Queue()
### State Declaration ###
#1: Disengage - start/stop control of the node

class Disengage(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['started', 'completed', 'aborted'])
    def execute(self,userdata) :     
        while not rospy.is_shutdown():
            if isEnd:
                rospy.signal_shutdown("Deactivating Acoustic Node")
                return 'completed'
            if isStart:
                print "#1 Starting Acoustic node"
                return 'started'
        return 'aborted'
#Waiting for a clean signal to come

class CollectTDOA(smach.State):
    def __init__(self,timeout=30,samples=5,farfield=True):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
    def execute(self,userdata):
        global tdoa_queue

        start_time = rospy.get_time()
        for i in range(samples):
            print 'waiting for #{0} good ping...'.format(i)
            start_time = rospy.get_time()
            while not new_data and (rospy.get_time() - start_time) < timeout:
                rospy.sleep(1)

            if new_data:
                print 'acquired #{0} good ping'.format(i)
                if farfield:
                    tdoa = far_field()
                else:
                    tdoa = nearfield()
                tdoa_queue.put(tdoa)
            else:
                print 'timed out! Cannot acquire any good ping'

#Collect 5 sets of clean signal, calculate angle/direction
def collectsignal(timeout=30,maxangle=130,samples=5,far_field=True):
    global state_heading
    global state0_heading
    global P_heading
    global P0_heading
    global R_heading
    global new_data
    global new_heading
    #(state_heading, P_Heading) = initialize_filter(state0_heading, P0_heading)
    state_heading = state0_heading
    P_heading = P0_heading
    for i in range(samples):
        print 'waiting for #{0} good ping'.format(i)
        start_time = rospy.get_time()
        while not new_data and (rospy.get_time() - start_time) < timeout:
            rospy.sleep(1)

        if new_data:
            print 'acquired #{0} good ping'.format(i)
            direction=far_field()
            rel_heading = math.atan2(direction.y,direction.x)*180/math.pi

            if np.fabs(rel_heading) < maxangle:
                print "new relative heading", rel_heading
                (state_heading, P_heading) = step_filter(state_heading, rel_heading, P_heading, R_heading)
                print "new filtered heading", state_heading
                print "current heading", yaw
            else:
                print 'suspect wrong angle! rel_heading = {0}'.format(rel_heading)
        else:
            print 'timed out! Cannot acquire any good ping'

#2: CorrectHeading - use far field equation
class CorrectHeading(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed','aborted'])
    def execute(self,userdata):
        print '#2 Correcting heading:'

        new_heading = (yaw +state_heading) % 360
        print "Correcting heading to {0:.3f}".format(new_heading)
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=0, heading_setpoint=new_heading, depth_setpoint=0.3, sidemove_setpoint=0)
        print "####"
        print goal
        print "####"
        movement_client.send_goal(goal)
        movement_client.wait_for_result()
                        
        #maintain current heading, depth 
        return 'completed'

#3: SearchAhead - keep current heading, search until certain condition to use near field equation
class SearchAhead(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed','aborted'])
    def execute(self,userdata):
        print '#3 moving forward'
        #New code
        global new_heading
        global search_depth
        global search_distance
        global z_threshold
        global x, y, z
        global new_data
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=2*search_distance, heading_setpoint=new_heading,depth_setpoint=search_depth,sidemove_setpoint=0)
        print "####"
        print goal
        print "####"
        movement_client.send_goal(goal)
        movement_client.wait_for_result()
        
        while z < z_threshold and not rospy.is_shutdown():
            while not new_data: #busy waiting to wait for new data
                pass
            new_data = False
            far_field()
            rel_heading = math.atan2(y,x) * 180.0/math.pi
            if np.fabs(rel_heading) > 130:
                print "Suspect wrong relative heading"
                continue
            new_heading = (yaw + rel_heading) % 360
            goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=search_distance, heading_setpoint=new_heading,depth_setpoint=search_depth,sidemove_setpoint=0)
            print "####"
            print goal
            print "####"
            movement_client.send_goal(goal)
            movement_client.wait_for_result()
            
            print "Finish searching, change to near field equation"
            return 'completed'
        #End new code
#4: DriveThru - use near file equation and find exact position of the pinger
class DriveThru(smach.State):
    global isStart,isEnd
    global search_depth
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed','aborted'])
    def execute(self,userdata):
        global meas_x, state_x, state0_x
        global P_x, P0_x, R_x
        global meas_y, state_y, state0_y
        global P_y, P0_y, R_y
        global x, y, z
        global new_data

        print '#4 Moving to exact position using near field'
        state_x = state0_x
        P_x = P0_x
        state_y = state0_y
        P_y = P0_y
        for i in range(5):
            while not new_data: #busy waiting to wait for new data
                pass
            new_data = False
            near_field()

            (state_x, P_x) = step_filter(state_x, x, P_x, R_x)
            (state_y, P_y) = step_filter(state_y, y, P_y, R_y)
            print "Collecting near field data: ", i
            print "x y:        ", x, y
            print "filtered x y", x, y
        
        rel_heading = math.atan2(y,x) * 180.0/math.pi
        print 'x={0:.3f},y={1:.3f},z={2:.3f},yaw={3:.3f},rel_heading={4:.3f}'.format(x,y,z,yaw,rel_heading)
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint=x,heading_setpoint=yaw,depth_setpoint=search_depth,sidemove_setpoint=y)
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

#Triangulation consts
speedOfSound = 1484
x_1, y_1, z_1 = 0  ,-0.07, 0.07 #left hydrophone
x_2, y_2, z_2 = 0  , 0.07, 0.07 #right hydrophone
x_3, y_3, z_3 = 0.1, 0   , 0    #top hydrophone

def far_field():
    global new_data
    new_data = False

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
    return (x,y,z)
    
def near_field(z=2.4):
    global new_data
    new_data = False
    
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
    return (x,y,z)

def step_filter(state, meas, P, R):
    K = P/(P + R)
    state = state + K * (meas - state)
    P = (1 - K) * P
    return (state, P)

def get_tdoa(msg):
    global d10,d20,d30
    global new_data
    new_data = True
    d10=msg.linear.x/1000000*speedOfSound
    d20=msg.linear.y/1000000*speedOfSound
    d30=msg.linear.z/1000000*speedOfSound

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


#Kalman filter variables
state_heading = 0
state0_heading = 0
P_heading = 0
P0_heading = 200
R_heading = 0.5

state_x = 0
state0_x = 0
P_x = 0
P0_x = 40
R_x = 0.1

state_y = 0
state0_y = 0
P_y = 0
P0_y = 40
R_y = 0.1

#whether new time diff values received
new_data = False
#calculated heading
new_heading = 0
#seach state parameters
search_depth = 0.3
search_distance = 1
z_threshold = 0.7

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
    rospy.Subscriber("/hydrophone/time_diff",Twist,get_tdoa)
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
