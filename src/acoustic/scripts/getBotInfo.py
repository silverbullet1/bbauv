#!/usr/bin/env python
import rospy
import roslib
from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from nav_msgs.msg import Odometry
import actionlib

abs_pos = {'x':0, 'y':0}
depth = 0.0
heading = {'yaw':0, 'pitch':0, 'roll': 0}

def stopRobot():
    sendMovement(f=0, sm=0)

def DVLcallback(data):
    global abs_pos
    abs_pos['x'] = data.pose.pose.position.x
    abs_pos['y'] = data.pose.pose.position.y

def compass_callback(data):
    global heading
    heading['yaw'] = data.yaw

def depth_callback(data):
    global depth
    depth = data.depth 


def sendMovement(f=0.0, h=None, sm=0.0, d=None):
    global locomotionClient
    goal = ControllerGoal(forward_setpoint=f, heading_setpoint=h, sidemove_setpoint=sm, depth_setpoint=d)
    locomotionClient.send_goal(goal)
    locomotionClient.wait_for_result(rospy.Duration(0.5))

def getBetterAngle(angle, heading):
    sendMovement(h = angle, d = depth)
#Subscibers
locomotionClient = actionlib.SimpleActionClient('LocomotionServer',ControllerAction) 
controllerSettings = rospy.ServiceProxy("/set_controller_srv", set_controller)
controllerSettings.wait_for_service()
navClient = rospy.ServiceProxy("/navigate2D", navigate2d)
navClient.wait_for_service()
rospy.ServiceProxy("/set_controller_srv", set_controller)
setServer = rospy.ServiceProxy("/set_controller_srv", set_controller)
#setServer(forward=True, sidemove=True, heading=True, depth=False, pitch=True, roll=False, topside=False, navigation=False)

abs_pos_sub = rospy.Subscriber("earth_odom", Odometry, DVLcallback)
depth_sub = rospy.Subscriber("/depth", depth, depth_callback)
compass_sub = rospy.Subscriber('/euler', compass_data, compass_callback)


if __name__ =="__main__":
    while True:
        print (str(abs_pos['x']) + "\n")
        print (str(abs_pos['y']) + "\n")
        print (str(depth) + "\n")
    #sendMovement(sm = 3.0, d = depth)
