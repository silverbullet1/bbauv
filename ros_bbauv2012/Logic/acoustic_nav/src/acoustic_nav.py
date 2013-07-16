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
    def __init__(self,timeout=10,samples=5):
        smach.State.__init__(self, outcomes=['succeeded','failed','aborted'])
        self.timeout=timeout
        self.samples=samples
    def execute(self,userdata):
        global tdoa_queue
        global farfield
        global isAbort
        global new_data

        for i in range(self.samples):
            start_time = rospy.get_time()
            while (not new_data and not isAbort and not rospy.is_shutdown()
                    and (rospy.get_time() - start_time) < self.timeout):
                print 'waiting for #{0} good ping...'.format(i + 1)
                rospy.sleep(1)

            if isAbort or rospy.is_shutdown():
                return 'aborted'
            if new_data:
                print 'acquired #{0} good ping'.format(i+1)
                tdoa = (d10,d20,d30)
                new_data=False
                tdoa_queue.put(tdoa)              
            else:
                print 'timed out! Cannot acquire any good ping'
                return 'failed'

        if isTest == False:
            try:
                resp = mission_srv_request(True, False, None)
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e

        return 'succeeded'

class CorrectHeading(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['farfield', 'nearfield', 'aborted'])
    def execute(self,userdata):
        global farfield
        global cur_heading
        global tdoa_queue
        z_threshold = 0.5
        
        if isAbort:
            return 'aborted'

        (rel_heading, z) = triangulate(tdoa_queue, "farfield")
        print 'rel_heading={0}, z={1})'.format(rel_heading,z)
        new_heading = (cur_heading + rel_heading) % 360
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint = 0, heading_setpoint = new_heading, depth_setpoint=0.3, sidemove_setpoint=0)
        print "-----------------------"
        print goal
        print "-----------------------"
        movement_client.send_goal(goal)
        movement_client.wait_for_result(action_timeout)

        cur_heading = new_heading

        if z < z_threshold:
            # continue far field, go to move forward
            farfield = True
            return 'farfield'
        else:
            # continue near field, go to collect TDOA 
            farfield = False
            return 'nearfield'

class GoToXY(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed', 'aborted'])
    def execute(self,userdata):
        global tdoa_queue
        global cur_heading

        if isAbort:
            return 'aborted'

        x, y = triangulate(tdoa_queue, "nearfield")
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint = x, heading_setpoint = cur_heading, depth_setpoint=search_depth, sidemove_setpoint=y)
        print "-----------------------"
        print goal
        print "-----------------------"
        movement_client.send_goal(goal)
        movement_client.wait_for_result(action_timeout)
        return 'completed'

class MoveForward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed', 'aborted'])
    def execute(self,userdata):
        if isAbort:
            return 'aborted'

        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint = search_distance, heading_setpoint = cur_heading, depth_setpoint=search_depth, sidemove_setpoint=0)
        print "-----------------------"
        print goal
        print "-----------------------"
        movement_client.send_goal(goal)
        movement_client.wait_for_result(action_timeout)
        return 'completed'

def far_field(d10, d20, d30):
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
    
def near_field(d10, d20, d30, z=2.4):   
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

def triangulate(data_queue, mode):
    queue_length = data_queue.qsize()
    if mode == "farfield":
        Ph = 200
        Rh = 0.5
        Pz = 40
        Rz = 0.1
        filter_heading = 0
        filter_z = 0
        for i in range(queue_length):
            #compute new measurement
            d10,d20,d30=data_queue.get()
            x, y, z = far_field(d10,d20,d30)
            rel_heading = math.atan2(y,x) * 180 / math.pi
            #filter steps
            Kh = Ph / (Ph + Rh)
            Kz = Pz / (Pz + Rz)
            filter_heading = filter_heading + Kh * (rel_heading - filter_heading)
            filter_z = filter_z + Kz * (z - filter_z)
            Ph = (1 - Kh) * Ph
            Pz = (1 - Kz) * Pz

            print '(rel_heading,z)=({0},{1})'.format(rel_heading,z)
        return filter_heading, filter_z

    else:
        Px = 40
        Rx = 0.1
        Py = 40
        Ry = 0.1
        filter_x = 0
        filter_y = 0
        for i in range(queue_length):
            d10,d20,d30=data_queue.get()
            x, y, z = near_field(d10,d20,d30)
            Kx = Px / (Px + Rx)
            Ky = Py / (Py + Ry)
            filter_x = filter_x + Kx * (x - filter_x)
            filter_y = filter_y + Ky * (y - filter_y)
            Px = (1 - Kx) * Px
            Py = (1 - Ky) * Py

        return filter_x, filter_y

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
#Triangulation consts
speedOfSound = 1484
x_1, y_1, z_1 = 0  ,-0.07, 0.07 #left hydrophone
x_2, y_2, z_2 = 0  , 0.07, 0.07 #right hydrophone
x_3, y_3, z_3 = 0.1, 0   , 0    #top hydrophone

#States variabes
isStart = False
isEnd = False
isAbort = False
isTest = True
movement_client = None
locomotionGoal = None 

action_timeout=rospy.Duration(2)
#Current state of vehicle:
#from Compass, DVL
yaw =0 
altitude =0
cur_heading=0
#from hydrophones (TDOA)
d10, d20, d30 = 0, 0, 0

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
    sm_top = smach.StateMachine(outcomes=['Task_completed','Task_failed','Aborted'])
    with sm_top:
        smach.StateMachine.add('DISENGAGE',Disengage(),
            transitions ={'started':'SEARCH','completed':'Task_completed','aborted':'Aborted'})
        
        smach.StateMachine.add('SEARCH',CollectTDOA(),
            transitions ={'succeeded':'CORRECTHEADING','failed':'Task_failed','aborted':'Aborted'})
        
        smach.StateMachine.add('CORRECTHEADING',CorrectHeading(),
            transitions ={'farfield':'MOVEFORWARD','nearfield':'GOTOXY','aborted':'Aborted'})
        
        smach.StateMachine.add('MOVEFORWARD',MoveForward(),
            transitions ={'completed':'SEARCH','aborted':'Aborted'})

        smach.StateMachine.add('GOTOXY',GoToXY(),
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
