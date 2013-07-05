#! /usr/bin/env python

import roslib; roslib.load_manifest('Vision')
import rospy
import math
import numpy as np
# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from geometry_msgs.msg import *

def doneCB(state,result):
    print "I'm done!"
    print result.forward_final;
    print result.heading_final;
    print result.sidemove_final;
    
def controller_client():
    print "Client!"
    # Waits until the action server has started up and started
    # listening for goals.
    
    # Waits for the server to finish performing the action.
    #client.wait_for_result()

def TDOA_sub_callback(msg):
    global time_diff1
    global time_diff2
    global time_diff3
    time_diff1=msg.linear.x/1000000
    time_diff2=msg.linear.y/1000000
    time_diff3=msg.linear.z/1000000

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
#global variables
d10=0
d20=0
d30=0
lamda = 0
x=0
y=0
z=0
yaw =0
#node constants
x_1=0
y_1=-0.07
z_1 = 0.07
x_2=0
y_2=0.07
z_2 = 0.07
x_3=0.1
y_3=0
z_3 =0
speedOfSound = 1484
#TDOA input
time_diff1 = 0
time_diff2 = 0
time_diff3 = 0
nearField = False
old_yaw = 0
if __name__ == '__main__':
    try:
        rospy.init_node('acoustic_navigation')
        client = actionlib.SimpleActionClient('LocomotionServer', bbauv_msgs.msg.ControllerAction)
        client.wait_for_server()
        
        rospy.Subscriber("/hydrophone/time_diff",Twist,TDOA_sub_callback)
        rospy.Subscriber("/euler",compass_data,get_heading)
        r=rospy.Rate(0.5) #run at 2Hz

        while not rospy.is_shutdown():
            d10 = time_diff1 * speedOfSound
            d20 = time_diff2 * speedOfSound
            d30 = time_diff3 * speedOfSound

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
