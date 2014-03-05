#!/usr/bin/env python
import rospy
import roslib
import actionlib
from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
import math
import socket 
import numpy as np

TCP_IP = '192.168.1.149'
TCP_PORT = 5100
BUFFER_SIZE = 60         
LOGFILE = "logger"
DEGREE_TWO_PI = 360
data = 0.0

class Test():
    global LOGFILE, sock
    def __init__(self):
        rospy.loginfo("Started node")
        self.heading = {'yaw':0, 'pitch':0, 'roll':0}
        self.pos = {'x':0, 'y':0}
        self.abs_pos = {'x':0, 'y':0}
        self.pointsCollected = []
        self.startSrv_Sub()
        #data = conn.recv(BUFFER_SIZE)
        while True:
            rospy.Rate(2)
            with open(LOGFILE, 'a') as f:
                f.write("Normalized heading: " + str(normaliseHeading(data, self.heading['yaw'] + "\n")))
                f.write("Vehicle Heading: " + str(self.heading['yaw']) +"\n")
                f.write("X: " + str(self.abs_pos['x'] + "\t\t\tY: " + str(self.abs_pos['y'] + "\n")))
                f.write("Num of Points collected: " + len(self.pointsCollected) + "\n")
                r.sleep()
                

    #Get all subscribers
    def startSrv_Sub(self):
    #Initialise Locomotion Client 
        try:
            self.locomotionClient = actionlib.SimpleActionClient('LocomotionServer',ControllerAction) 

        except rospy.ServiceException:
            rospy.logerr("Error running Locmotion Client")
    #Initialise Controller Service:
        try:
            self.controllerSettings = rospy.ServiceProxy("/set_controller_srv", set_controller)
            self.controllerSettings.wait_for_service()
        except rospy.ServiceException:
            rospy.logerr("Failed to connect to Controller")
    #Initialise Navigation Service
        try:
            self.navClient = rospy.ServiceProxy("/navigation2D", navigation2D, self.navCallback())
            self.navClient.wait_for_service()
        except rospy.ServiceException:
            rospy.logerr("Error connecting to navigate2D server")

    #Initialise connection to mission planner
        self.comServer = rospy.Service("/acoustic/mission_to_vision", mission_to_vision, self.handleSrv)

    #Initialise acoustic to mission 
        try:
            self.toMission = rospy.ServiceProxy("/acoustic/vision_to_mission", vision_to_mission)
            self.toMission.wait_for_service(timeout = 5)
        except rospy.ServiceException:
            rospy.logerr("Error connecting to mission planner")

    #Setting controller server
        setServer = rospy.ServiceProxy("/set_controller_srv", set_controller)
        setServer(forward=True, sidemove=True, heading=True, depth=False, pitch=True, roll=False,
                  topside=False, navigation=False)

    #Initialise publisher
        self.angle_pub = rospy.Publisher('/acoustic/angleFromPing', acoustic)

    #Initialise pos_sub
        self.position_sub = rospy.Subscriber("/WH_DVL_data", Odometry ,self.pos_callback)

    #Initialise earth_odom
        self.abs_pos_sub = rospy.Subscriber("earth_odom", Odometry, self.DVL_callback)

    #Initialise depth_sub
        self.depth_sub = rospy.Subscriber("/depth", depth, self.depth_callback)

    #Inialise compass_sub 
        self.compass_sub = rospy.Subscriber('/euler', compass_data, self.compass_callback)

    #Callbacks

    def DVL_callback(self, data):
        self.abs_pos['x'] = data.pose.pose.x
        self.abs_pos['y'] = data.pose.pose.y


    def navCallback(self,res):
        rospy.loginfo("Sent x and y coordinate for navigation")

    def compass_callback(self,data):
        self.heading['yaw'] = data.yaw

    def depth_callback(self,data):
        self.depth = data.depth 

    def pos_callback(self,data):
        self.pos['x'] = data.pose.pose.position.x
        self.pos['y'] = data.pose.pose.position.y

    def connectToLabView(self):
        global TCP_IP, TCP_PORT, BUFFER_SIZE, LOGFILE, data
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        while True:
            self.sock.bind(TCP_IP, TCP_PORT)
            self.sock.listen(1)
            (conn, addr) = self.sock.accept()	
            data = conn.recv(BUFFER_SIZE)
        conn.close()

    def normaliseHeading(self, angle, heading):
        global DEGREE_TWO_PI
        return (DEGREE_TWO_PI+heading+angle) % (DEGREE_TWO_PI)

if __name__ == "__main__":
    rospy.init_node("logger")
    test = Test()
    self.connectToLabView()
    

