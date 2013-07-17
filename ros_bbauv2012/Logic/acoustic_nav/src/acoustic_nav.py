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
                rospy.loginfo('Starting Acoustic node')
                return 'started'
            rospy.sleep(1)
        return 'aborted'
#Waiting for a clean signal to come

class CollectTDOA(smach.State):
    def __init__(self,samples=5,timeout=15):
        smach.State.__init__(self, outcomes=['succeeded','failed','aborted'])
        self.timeout=timeout
        self.samples=samples
    def execute(self,userdata):
        global tdoa_queue
        global farfield
        global isAbort
        global isStart
        global new_data
        new_data=False
        while (not tdoa_queue.empty()):
            tdoa_queue.get()
        for i in range(self.samples):
            start_time = rospy.get_time()
            while (not new_data and not isAbort and not rospy.is_shutdown()
                    and (rospy.get_time() - start_time) < self.timeout):
                rospy.sleep(1)

            if isAbort or rospy.is_shutdown():
                return 'aborted'
            if new_data:
                rospy.loginfo('acquired #{0} good ping'.format(i+1))
                tdoa = (d10,d20,d30)
                new_data=False
                tdoa_queue.put(tdoa)              
            else:
                isStart=False
                rospy.loginfo('timed out! Cannot acquire any good ping')
                return 'failed'

        if isTest == False:
            try:
                resp = mission_srv_request(True, False, None)
            except rospy.ServiceException, e:
                rospy.loginfo("Service call failed: %s" % e)

        return 'succeeded'

class CorrectHeading(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['farfield', 'nearfield', 'aborted'])
    def execute(self,userdata):
        global farfield
        global cur_heading
        global tdoa_queue
        z_threshold = 0.7
        
        if isAbort:
            return 'aborted'
        rospy.loginfo('yaw = {0}, cur_heading = {1}'.format(yaw,cur_heading))
        (rel_heading, z) = triangulate(tdoa_queue, "farfield")
        rospy.loginfo('yaw = {0}, cur_heading = {1}'.format(yaw,cur_heading))
        rospy.loginfo('==>rel_heading={0}, z={1}'.format(rel_heading,z))
        if np.fabs(rel_heading) > 10:
            rospy.loginfo("correcting heading....")
            new_heading = (cur_heading + rel_heading) % 360
            goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint = 0, heading_setpoint = new_heading, depth_setpoint=search_depth, sidemove_setpoint=0)
            print "-----------------------"
            print goal
            print "-----------------------"
            movement_client.send_goal(goal)
            movement_client.wait_for_result(action_timeout)
            cur_heading = new_heading
        else:
            rospy.loginfo("good heading! maintain current heading")
             
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
        
        if isTest == False:
            try:
                ctrl = controller()
                ctrl.depth_setpoint = search_depth
                ctrl.heading_setpoint = cur_heading
                resp = mission_srv_request(False, True, ctrl)
            except rospy.ServiceException, e:
                rospy.loginfo("Service call failed: %s" % e)

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
    z=ver_dist_to_pinger
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

            rospy.loginfo('(rel_heading,z)=({0},{1})'.format(rel_heading,z))
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

            rospy.loginfo('(x,y)=({0},{1})'.format(x,y))
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
    global cur_heading
    yaw=msg.yaw

def get_altitude(msg):
    global altitude
    altitude=msg.data
def handle_srv(req):
    global isStart
    global isAbort
    global locomotionGoal
    global cur_heading
    global first_call
    global search_depth
    rospy.loginfo("Acoustic service handled.")
    if req.start_request:
        rospy.loginfo("isStart true.")
        isStart = True
        isAbort = False
        # Format for service: start_response, abort_response
        locomotionGoal = req.start_ctrl
        if first_call:
            first_call=False
            cur_heading=locomotionGoal.heading_setpoint
            search_depth=locomotionGoal.depth_setpoint

    if req.abort_request:
        rospy.loginfo("Acoustic abort received")
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
isTest = False
movement_client = None
locomotionGoal = None 
first_call = True
action_timeout=rospy.Duration(30)
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
search_depth = 0.4
search_distance = 2
z_threshold = 0.6
ver_dist_to_pinger = 2.41 

if __name__ == '__main__':
    rospy.init_node('acoustic_navigation')
    loop_rate=rospy.Rate(0.5)

    rospy.loginfo("AcousticNavigation activated!")
    rospy.wait_for_service('set_controller_srv')
    set_controller_request=rospy.ServiceProxy('set_controller_srv',set_controller)
    set_controller_request(True,True,True,True,False,False,False)
     
    #Setup movement client
    movement_client = actionlib.SimpleActionClient('LocomotionServer', bbauv_msgs.msg.ControllerAction)
    movement_client.wait_for_server()
    #Setup acoustic service 
    #pls rename mission-to-vision to mission-to-task!!!
    #acoustic_srv = rospy.service('acoustic_srv',mission_to_vision,handle_srv)

    if isTest:
        isStart= True
        cur_heading=10000 #dummy val
    else:
        #setup client
        rospy.loginfo('waiting for mission_srv...')
        rospy.wait_for_service('mission_srv') #connecting to mission server
        mission_srv_request = rospy.ServiceProxy('mission_srv', vision_to_mission)
        rospy.loginfo('connected to mission_srv!')
        #setup server
        acoustic_srv = rospy.Service('acoustic_srv',mission_to_vision,handle_srv) 
        rospy.loginfo('acoustic_srv initialized')

    #Setup Subscribers
    rospy.Subscriber("/hydrophone/time_diff",Twist,get_tdoa)
    rospy.Subscriber("/euler",compass_data,get_heading)
    rospy.Subscriber("/altitude",Float32,get_altitude)

    #State Machines
    sm_top = smach.StateMachine(outcomes=['Task_completed','Task_failed'])
    with sm_top:
        smach.StateMachine.add('DISENGAGE',Disengage(),
            transitions ={'started':'INITIALSEARCH','completed':'Task_completed','aborted':'DISENGAGE'})
        
        smach.StateMachine.add('INITIALSEARCH',CollectTDOA(samples=3),
            transitions ={'succeeded':'CORRECTHEADING','failed':'DISENGAGE','aborted':'DISENGAGE'})

        smach.StateMachine.add('CORRECTHEADING',CorrectHeading(),
            transitions ={'farfield':'MOVEFORWARD','nearfield':'NEARSEARCH','aborted':'DISENGAGE'})

        smach.StateMachine.add('MOVEFORWARD',MoveForward(),
            transitions ={'completed':'FARSEARCH','aborted':'DISENGAGE'})
        
        smach.StateMachine.add('FARSEARCH',CollectTDOA(samples=3),
            transitions ={'succeeded':'CORRECTHEADING','failed':'DISENGAGE','aborted':'DISENGAGE'})
        
        smach.StateMachine.add('NEARSEARCH',CollectTDOA(samples=5),
            transitions ={'succeeded':'GOTOXY','failed':'DISENGAGE','aborted':'DISENGAGE'})
         
        smach.StateMachine.add('GOTOXY',GoToXY(),
            transitions ={'completed':'Task_completed','aborted':'DISENGAGE'})

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
