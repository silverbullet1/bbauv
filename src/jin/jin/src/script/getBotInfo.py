#!/usr/bin/env python
import rospy
import roslib
from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from nav_msgs.msg import Odometry
import actionlib
import random


class Test():
    heading = {'yaw':0, 'pitch':0, 'roll': 0}
    def __init__(self):
        rospy.init_node("test")
        self.abs_pos = {'x':0, 'y':0}
        self.depth = 0.0
        self.heading = {'yaw':0, 'pitch':0, 'roll': 0}
        self.pub = rospy.Publisher("acoustic/angFromPing", acoustic) 

        #Subscibers
        try:
            self.locomotionClient = actionlib.SimpleActionClient('LocomotionServer', ControllerAction) 
            rospy.loginfo("Waiting for Locomotion Server")
            self.locomotionClient.wait_for_server(timeout=rospy.Duration(5))
        except Exception as e:
            rospy.loginfo(e)
        rospy.loginfo("Got Locomotion Server")

        self.controllerSettings = rospy.ServiceProxy("/set_controller_srv", set_controller)
        self.controllerSettings.wait_for_service()
        #self.navClient = rospy.ServiceProxy("/navigate2D", navigate2d)
        #self.navClient.wait_for_service()
        self.controllerSettings(forward=True, sidemove=True, heading=True, depth=True, pitch=True, roll=False, topside=False, navigation=False)

        self.abs_pos_sub = rospy.Subscriber("earth_odom", Odometry, self.DVLcallback)
        self.depth_sub = rospy.Subscriber("/depth", bbauv_msgs.msg.depth, self.depth_callback)
        self.compass_sub = rospy.Subscriber('/euler', bbauv_msgs.msg.compass_data, self.compass_callback)

    def stopRobot(self):
        self.sendMovement(f=0, sm=0)

    def DVLcallback(self, data):
        self.abs_pos['x'] = data.pose.pose.position.x
        self.abs_pos['y'] = data.pose.pose.position.y

    def compass_callback(self, data):
        self.heading['yaw'] = data.yaw
        print self.heading['yaw']

    def depth_callback(self, data):
        self.depth = data.depth 

    def sendMovement(self, f=0.0, h=None, sm=0.0, d=None):
        d = d if d else self.depth
        h = h if h else self.heading['yaw']
        thienisgay = ControllerGoal(forward_setpoint=f, heading_setpoint=h, sidemove_setpoint=sm, depth_setpoint=d)
        self.locomotionClient.send_goal(thienisgay)
        self.locomotionClient.wait_for_result()

    def getBetterAngle(self, angle):
        self.sendMovement(h = angle, d = depth)

    def display(self):
        print ("X_Pos: " + str(self.abs_pos['x']) + "\n")
        print ("Y_Pos: " + str(self.abs_pos['y']) + "\n")
        print ("Depth: " + str(self.depth) + "\n")
        print ("Heading: " + str(self.heading['yaw']) + "\n")

if __name__ =="__main__":
    a = Test()
    #rospy.sleep(1)
    #rospy.loginfo(a.heading['yaw'])
    #a.sendMovement(h=a.heading['yaw'] + 0)
    #rospy.loginfo("Turned Nothing")
    a.sendMovement(a.heading['yaw'])
    a.sendMovement(f=2)
    rospy.loginfo("Forward")
    a.sendMovement(h=(a.heading['yaw'] + 30)%360)
    rospy.loginfo("Turned")
    a.sendMovement(f=2)
    rospy.loginfo("FOrward")
    while not rospy.is_shutdown():
        a.display()
    rospy.spin()
    #stopRobot()
