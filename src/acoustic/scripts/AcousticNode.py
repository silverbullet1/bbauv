#!/usr/bin/env python
import rospy, roslib
from nav_msgs.msg import Odometry
from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
import socket
import actionlib
from numpy import matrix
import math

roslib.load_manifest('vision')

#Constants
HAS_FIRST_POS = False
x1_coordinate = y1_coordinate = aoa1_value = 0
x2_coordinate = y2_coordinate = aoa2_value = 0
xp_coordinate = yp_coordinate = 0
bearingAtPoint1 = bearingAtPoint2 = 0
DEGREE_TWO_PI = 360
DEGREE_PI = 180
TCP_IP = '192.168.1.149'
TCP_PORT = 5085
BUFFER_SIZE = 25         

class AcousticNode(object):


    def __init__(self):
        rospy.loginfo("Entering the zone")
        self.angle = 0
        self.heading = {'yaw':0, 'pitch':0, 'roll':0}
        self.depth = 0
        self.pos = {'x':0, 'y':0}
        self.server = rospy.Service('/acoustic', acoustic,self.handlerAcoustic)
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
    #Initialise pos_sub
        self.position_sub = rospy.Subscriber("/WH_DVL_data", Odometry ,self.pos_callback)

    #Initialise depth_sub
        self.depth_sub = rospy.Subscriber("/depth", depth, self.depth_callback)

    #Inialise compass_sub 
        self.compassSub = rospy.Subscriber('/euler', compass_data, self.compass_callback)

    #Methods 

    def navCallback(self,res):
        rospy.loginfo("Sent x and y coordinate for navigation")

    def compass_callback(self,data):
        self.heading['yaw'] = data.yaw

    def depth_callback(self,data):
        self.depth = data.depth 

    def pos_callback(self,data):
        self.pos['x'] = data.pose.pose.position.x
        self.pos['y'] = data.pose.pose.position.y

    def sendMovement(self, f=0.0, h=None, sm=0.0, d=None):
        goal = ControllerGoal(forward_setpoint=f, heading_setpoint=h, sidemove_setpoint=sm, depth_setpoint=d)
        self.locomotionClient.send_goal(goal)
        self.locomotionClient.wait_for_result(rospy.Duration(0.5))

    def getPoints(self):
        global x1_coordinate, x2_coordinate, y1_coordinate, y2_coordinate,bearingAtPoint1,bearingAtPoint2
        if self.withinRange(self.angle):
            if not HAS_FIRST_POS:
                x1_coordinate = self.pos['x']
                x2_coordinate = self.pos['y']
                bearingAtPoint1 = self.heading['yaw']
                sidemove = 1 if self.angle < 0 else -1
                self.navClient(self.pos['x'], self.pos['y'] + sidemove)              
                self.getPoints()
            else:
                x2_coordinate = self.pos['x']
                y2_coordinate = self.pos['y']
                bearingAtPoint2 = self.heading['yaw']
                rospy.sleep(1000) 
        else:
            #Turn heading to make sure the pinger is within nice range
            newHeading = (self.angle + 30)/2 if self.angle < 0 else (self.angle - 30)/2
            self.sendMovement(h = newHeading)
                    
    def handlerAcoustic(self,req):
        return acousticResponse(xp_coordinate,yp_coordinate)

    def splitMsg(self,data):
        global x1_coordinate, y1_coordinate, aoa1_value
        global x2_coordinate, y2_coordinate, aoa2_value
        global bearingAtPoint1, bearingAtPoint2
        global xp_coordinate, yp_coordinate
        bearingAtPoint1 = bearingAtPoint2 = 0
        (x1_coordinate, y1_coordinate, aoa1_value, x2_coordinate, y2_coordinate, aoa2_value) = data.split('.')

    def triangulate(self):
        #converting all data points to numeric
        #Compensate bearingAtPoint according to True North
        global x1_coordinate, y1_coordinate, aoa1_value    
        global x2_coordinate, y2_coordinate, aoa2_value
        global bearingAtPoint1, bearingAtPoint2
        global xp_coordinate, yp_coordinate
        x1_coordinate 	= int(x1_coordinate)
        y1_coordinate 	= int(y1_coordinate)
        aoa1_value	= int(aoa1_value)
        x2_coordinate	= int(x2_coordinate)
        y2_coordinate	= int(y2_coordinate)
        aoa2_value	= int(aoa2_value)
        bearingAtPoint1 = float((DEGREE_TWO_PI+bearingAtPoint1+aoa1_value) % (DEGREE_TWO_PI))
        bearingAtPoint2 = float((DEGREE_TWO_PI+bearingAtPoint2+aoa2_value) % (DEGREE_TWO_PI))

        if (((math.sin(bearingAtPoint1/DEGREE_PI*math.pi)) != 0) and ((math.sin(bearingAtPoint2/DEGREE_PI*math.pi)) != 0)):	
            m1 =  ((math.cos(bearingAtPoint1/DEGREE_PI*math.pi)) / (math.sin(bearingAtPoint1/DEGREE_PI*math.pi)))	
            c1 = y1_coordinate - (m1*x1_coordinate)
            m2 =  ((math.cos(bearingAtPoint2/DEGREE_PI*math.pi)) / (math.sin(bearingAtPoint2/DEGREE_PI*math.pi)))
            c2 = y2_coordinate - (m2*x2_coordinate)
            constant_matrix    = matrix([[c1],[c2]])
            coefficient_matrix = matrix([[1,m1],[1,m2]])
            result_matrix 	   = coefficient_matrix.I * constant_matrix
            xp_coordinate = result_matrix[0,0]                      
            yp_coordinate = result_matrix[1,0]                                      
        else:
            print "Division by zero has occured"                                  
            xp_coordinate = "-"                          
            yp_coordinate = "-"                                     

    def withinRange(self, angle):
        return angle >= -30 and angle <= 30 


if __name__ == "__main__":
    try:
        rospy.init_node("acoustic")
        acoustic = AcousticNode()
        rospy.spin()
        while True:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.bind((TCP_IP, TCP_PORT))
            s.listen(1)
            (conn, addr) = s.accept()	
            data = conn.recv(BUFFER_SIZE)
            if data == "stop":
                print ("Program was told to be stopped")
                break
            else:
                if test:
                    self.splitMsg(data)
                    self.triangulate()
                    print xp_coordinate, yp_coordinate
                else:
                    rospy.loginfo("Waiting for request from mission planner")
                    self.angle = data
                    self.getPoints()
                    self.triangulate()
    except KeyboardInterrupt:
        conn.close()
        rospy.logerr("Somebody killed me")
