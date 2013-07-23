#! /usr/bin/env python

import roslib; roslib.load_manifest('acoustic_nav')
import rospy
import actionlib
import smach
import smach_ros

from dynamic_reconfigure.server import Server
from acoustic_nav.cfg import acoustic_navConfig

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

from Queue import Queue
import math
import numpy as np

### State Declaration ###
''' Disengage state - simply wait =)))))
'''
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

'''CollectTDOA state - collect a few signal to avoid random error, store data in a queue
'''
class CollectTDOA(smach.State):
    def __init__(self,samples=5,timeout=20,initSearch=False):
        smach.State.__init__(self, outcomes=['succeeded','failed','aborted'])
        self.timeout=timeout
        self.samples=samples
        self.initSearch=initSearch
    def execute(self,userdata):
        global tdoa_queue
        global farfield
        global isAbort
        global isStart
        global new_data
        global recover_time

        new_data=False
        while (not tdoa_queue.empty()):
            tdoa_queue.get()
        rospy.loginfo('Current Pose: x={0:.2f},y={1:.2f},z={2:.2f},yaw={3:.2f}'
                    .format(search_position.x,search_position.y,search_depth,cur_heading))
        for i in range(self.samples):
            start_time = rospy.get_time()
            rospy.loginfo('waiting for #{0} ping...'.format(i+1))
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
                break   
                        
        else: #if for loop is not broken!
            if isTest == False and self.initSearch == True:
                try:
                ###CHANGE HERE!!!!!!!!!!! Change what??!
                    resp = mission_srv_request(True, None, None)
                    rospy.loginfo('Found Good Ping! Grab control from mission. Send request: True None None')
                except rospy.ServiceException, e:
                    rospy.loginfo("Service call failed: %s" % e)

            recover_time = 0
            return 'succeeded'

        #if for loop is broken!    
        rospy.loginfo('timed out! Cannot acquire any good ping')
        if isTest == False and self.initSearch == True:
            try:
                resp = mission_srv_request(False, None, None)
                rospy.loginfo('Faied to acquire Good Ping! Return control to mission. Send request: False None None')
            except rospy.ServiceException, e:
                rospy.loginfo("Service call failed: %s" % e)
        return 'failed'

''' CorrectHeading state - correct the heading base on far field equation
'''
class CorrectHeading(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['farfield', 'nearfield', 'aborted'])
    def execute(self,userdata):
        global farfield
        global cur_heading
        global tdoa_queue
        global new_data
        
        if isAbort:
            return 'aborted'
        
        tdoa_backup=tdoa_queue

        (rel_heading, z) = triangulate(tdoa_queue, "farfield")
        rospy.loginfo('==> rel_heading= {0:.2f}, z= {1:.2f}'.format(rel_heading,z))
        new_heading = (cur_heading + rel_heading) % 360

        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint = 0, heading_setpoint = new_heading, depth_setpoint=search_depth, sidemove_setpoint=0)
        rospy.loginfo(goal)
        movement_client.send_goal(goal)
        movement_client.wait_for_result(action_timeout)
        cur_heading = new_heading
        new_data = False
             
        if math.fabs(z) < z_threshold:
            # continue far field, go to move forward
            farfield = True
            return 'farfield'
        else:
            # continue near field, go to collect TDOA
            rospy.sleep(3) 
            rospy.loginfo("Waiting 3s for clean signal")
            farfield = False
            return 'nearfield'

'''GoToXY state - go to a specific relative location calculated by near field equation
'''
class GoToXY(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed', 'aborted'])
    def execute(self,userdata):
        global tdoa_queue
        global cur_heading
        global isAbort
        global isStart
        if isAbort:
            return 'aborted'

        x, y = triangulate(tdoa_queue, "nearfield")
        x+=x_offset
        y+=y_offset
        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint = x, heading_setpoint = cur_heading, depth_setpoint=search_depth, sidemove_setpoint=y)
        rospy.loginfo(goal)
        movement_client.send_goal(goal)
        movement_client.wait_for_result(action_timeout)
        
        if isTest == False:
            try:
                ctrl = controller()
                ctrl.depth_setpoint = search_depth
                ctrl.heading_setpoint = cur_heading
                if not isAbort:
                    resp = mission_srv_request(None, True, ctrl)
                    rospy.loginfo('Task completed! Finished MovetoXY. Send request: None True Ctrl')
                    rospy.loginfo('Controller: %s'%ctrl)
                    isAbort = True
                    isStart = False
            except rospy.ServiceException, e:
                rospy.loginfo("Service call failed: %s" % e)
        return 'completed'

'''MoveForward state - Simply move forward =))))) 
'''
class MoveForward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed', 'aborted'])
    def execute(self,userdata):
        if isAbort:
            return 'aborted'

        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint = search_distance, heading_setpoint = cur_heading, depth_setpoint=search_depth, sidemove_setpoint=0)
        rospy.loginfo(goal)
        movement_client.send_goal(goal)
        #movement_client.wait_for_result(action_timeout)
        return 'completed'

'''Recover state - Try to recover when received bad signal
'''
class Recover(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed', 'aborted'])
    def execute(self,userdata):
        global cur_heading
        global recover_time
        global isAbort
        global isStart
        if isAbort: 
            return 'aborted'
        if recover_time == 3:
            rospy.loginfo('Unable to recover. Abort and go to Disengage')
            if not isTest:
                try:
                    ctrl = controller()
                    ctrl.depth_setpoint = search_depth
                    ctrl.heading_setpoint = cur_heading
                    if not isAbort:
                        resp = mission_srv_request(None, False, ctrl)
                        isAbort = True
                        isStart = False
                        rospy.loginfo('Task Completed! Failed at Recovering stage. Send request: None False Ctrl')
                        rospy.loginfo('Controller: %s' %ctrl)
                except rospy.ServiceException, e:
                    rospy.loginfo("Service call failed: %s" % e)
            return 'aborted'

        recover_time += 1
        if recover_time == 1:
            cur_heading = (cur_heading + 90) % 360
        elif recover_time == 2:
            cur_heading = (cur_heading + 180) % 360
        elif recover_time == 3:
            cur_heading = (cur_heading - 90) % 360

        rospy.loginfo("#{0} attempt to recover....".format(recover_time))

        goal = bbauv_msgs.msg.ControllerGoal(forward_setpoint = 0, heading_setpoint = cur_heading, depth_setpoint=search_depth, sidemove_setpoint=0)
        rospy.loginfo(goal)
        movement_client.send_goal(goal)
        movement_client.wait_for_result(action_timeout)
         
        return 'completed'

'''far field equation
'''
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

'''near field equaiton
'''
def near_field(d10, d20, d30, z=2.4):   
    z=ver_dist_to_pinger
    x,y=0,0
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
    else:
        z=-1
    return (x,y,z)

''' triangulate state - get data from the queue, calculate position/ direction then apply filter
'''
def triangulate(data_queue, mode):
    queue_length = data_queue.qsize()
    if mode == "farfield":
        Ph = 200
        Rh = 3
        Pz = 40
        Rz = 0.1
        filter_heading = 0
        filter_z = 0
        rel_heading = 0
        for i in range(queue_length):
            #compute new measurement
            d10,d20,d30=data_queue.get()
            x, y, z = far_field(d10,d20,d30)
            if z > 0:
                rel_heading = math.atan2(y,x) * 180 / math.pi
                #filter steps
                Kh = Ph * 1.0 / (Ph + Rh)
                Kz = Pz * 1.0 / (Pz + Rz)
                filter_heading = filter_heading + Kh * (rel_heading - filter_heading)
                filter_z = filter_z + Kz * (z - filter_z)
                Ph = (1 - Kh) * Ph
                Pz = (1 - Kz) * Pz

                rospy.loginfo('(heading,z)=({0:.2f},{1:.2f})-->(f_heading={2:.2f},f_z={3:.2f})'.format(rel_heading,z,filter_heading,filter_z))
            else:
                rospy.loginfo('(heading,z)=({0:.2f},{1:.2f})--> Invalid! z<0)'.format(rel_heading,z))
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
            #normalize z for filtering
            z = z/math.sqrt(x**2 + y**2 + z**2)
            if z > z_threshold: 
                Kx = Px * 1.0 / (Px + Rx)
                Ky = Py * 1.0 / (Py + Ry)
                filter_x = filter_x + Kx * (x - filter_x)
                filter_y = filter_y + Ky * (y - filter_y)
                Px = (1 - Kx) * Px
                Py = (1 - Ky) * Py
            
                rospy.loginfo('(x,y,z)=({0:.2f},{1:.2f},{2:.2f})-->(f_x,f_y)=({3:.2f},{4:.2f})'.format(x,y,z,filter_x,filter_y))
            else:
                rospy.loginfo('(x,y,z)=({0:.2f},{1:.2f},{2:.2f}) --> Invalid! z>{3:.2f}'.format(x,y,z,z_threshold))
        return filter_x, filter_y

def get_tdoa(msg):
    global d10,d20,d30
    global new_data
    new_data = True
    d10=msg.linear.x/1000000*speedOfSound
    d20=msg.linear.y/1000000*speedOfSound
    d30=msg.linear.z/1000000*speedOfSound
    x,y,z=far_field(d10,d20,d30)
    rel_heading = math.atan2(y,x) * 180 / math.pi
    
    rospy.loginfo('t10={0:.2f},t20={1:.2f},t30={2:.2f}'.format(msg.linear.x,msg.linear.y,msg.linear.z))
    rospy.loginfo('(x,y,z)=({0:.2f},{1:.2f},{2:.2f}), rel_heading={3:.2f}'.format(x,y,z,rel_heading))

def get_heading(msg):
    global yaw
    global cur_heading
    yaw=msg.yaw
    if (cur_heading==10000):
        cur_heading=yaw
def get_position(msg):
    global search_position 
    search_position=msg.pose.pose.position

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
        cur_heading=locomotionGoal.heading_setpoint
        search_depth=locomotionGoal.depth_setpoint

    if req.abort_request:
        rospy.loginfo("Acoustic abort received")
        isAbort = True
        isStart = False
    return mission_to_visionResponse(isStart, isAbort)

################### MAIN #####################
tdoa_queue = Queue()
recover_time = 0

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
search_depth = 0.5
search_distance = 3
z_threshold = 0.5
ver_dist_to_pinger = 1.73
x_offset = 0.27 #position of hydrophones
y_offset = -0.065
search_position = Point()

def dynamic_reconfigure_cb(config,level):
    global search_depth
    global search_distance
    global z_threshold
    global isTest
    global isAbort
    global ver_dist_to_pinger
    global x_offset
    global y_offset
    print 'geting parameter from dynamic reconfigure...' 
    isTest = config['isTest']
    isAbort= config['isAbort']
    x_offset=config['x_offset']
    y_offset=config['y_offset']
    search_depth=config['search_depth']
    search_distance=config['search_distance']
    z_threshold =config['z_threshold']
    ver_dist_to_pinger=config['altitude_at_pinger']-0.5+0.13 
    #1.13: dist from hydrophone to DVL, 0.5, height of pinger
    return config
if __name__ == '__main__':
    rospy.init_node('acoustic_navigation')
    loop_rate=rospy.Rate(0.5)

    rospy.loginfo("AcousticNavigation activated!")
    rospy.wait_for_service('set_controller_srv')
    
    #Setup dynamic reconfigure

    reconfigure_srv=Server(acoustic_navConfig, dynamic_reconfigure_cb)

    #Setup movement client
    movement_client = actionlib.SimpleActionClient('LocomotionServer', bbauv_msgs.msg.ControllerAction)
    movement_client.wait_for_server()

    if isTest:
        isStart= True
        cur_heading=10000 #dummy val
        set_controller_request=rospy.ServiceProxy('set_controller_srv',set_controller)
        set_controller_request(True,True,True,True,False,False,False)
    else:
        #setup client
        rospy.loginfo('waiting for mission_srv...')
        rospy.wait_for_service('mission_srv') #connecting to mission server
        mission_srv_request = rospy.ServiceProxy('mission_srv', vision_to_mission, headers={'id':'6'})
        rospy.loginfo('connected to mission_srv!')
        
        #setup server
        acoustic_srv = rospy.Service('acoustic_srv',mission_to_vision,handle_srv) 
        rospy.loginfo('acoustic_srv initialized')

    #Setup Subscribers
    rospy.Subscriber("/hydrophone/time_diff",Twist,get_tdoa)
    rospy.Subscriber("/euler",compass_data,get_heading)
    rospy.Subscriber("/altitude",Float32,get_altitude)
    rospy.Subscriber("/earth_odom",Odometry,get_position)

    #State Machines
    sm_top = smach.StateMachine(outcomes=['Task_completed','Task_failed'])
    with sm_top:
        smach.StateMachine.add('DISENGAGE',Disengage(),
            transitions ={'started':'INITIALSEARCH','completed':'Task_completed','aborted':'DISENGAGE'})
        
        smach.StateMachine.add('INITIALSEARCH',CollectTDOA(samples=3,timeout=20,initSearch=True),
            transitions ={'succeeded':'CORRECTHEADING','failed':'DISENGAGE','aborted':'DISENGAGE'})

        smach.StateMachine.add('CORRECTHEADING',CorrectHeading(),
            transitions ={'farfield':'MOVEFORWARD','nearfield':'NEARSEARCH','aborted':'DISENGAGE'})

        smach.StateMachine.add('MOVEFORWARD',MoveForward(),
            transitions ={'completed':'FARSEARCH','aborted':'DISENGAGE'})
        
        smach.StateMachine.add('FARSEARCH',CollectTDOA(samples=3,timeout=20),
            transitions ={'succeeded':'CORRECTHEADING','failed':'RECOVER','aborted':'DISENGAGE'})
        
        smach.StateMachine.add('NEARSEARCH',CollectTDOA(samples=5,timeout=20),
            transitions ={'succeeded':'GOTO_XY','failed':'RECOVER','aborted':'DISENGAGE'})
         
        smach.StateMachine.add('GOTO_XY',GoToXY(),
            transitions ={'completed':'DISENGAGE','aborted':'DISENGAGE'})
    
        smach.StateMachine.add('RECOVER',Recover(),
            transitions ={'completed':'FARSEARCH','aborted':'DISENGAGE'})

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
