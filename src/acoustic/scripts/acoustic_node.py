#!/usr/bin/env python

#Beware of radian and degree error 
#Make sure normalise bearing 
#Ensure sub stays stationary while taking angle
#Single Value Decomposition for passive reading

import rospy
import roslib; roslib.load_manifest('acoustic')
from nav_msgs.msg import Odometry
from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
import numpy as np
import actionlib
import math
import socket
import signal
from numpy import linalg as LA
from numpy import *
import math
import cmath
import array


#Constants
DEGREE_TWO_PI = 360
DEGREE_PI = 180
BUFFER_SIZE = 25

class Point():
    heading = 0
    pos = {'x': 0, 'y':0}
    angFromPing = 0

    def __init__(self, x, y, heading, angle):
        self.heading = heading
        self.pos['x'] = x
        self.pos['y'] = y
        self.angFromPing = angle


class AcousticNode(object):

    def __init__(self, param = {'test':False,'boundary':30,'sidemove':1,'takes':5,'active':False, 'takes': 5}):

        #ROS Params

        self.test = param['test']
        self.boundary = param['boundary']
        self.sidemove = param['sidemove']
        self.takes = param['takes']
        self.active = param['active']
        signal.signal(signal.SIGINT, self.quitProgram)


        self.data = []
        
        self.depth = 0
        self.heading = {'yaw':0, 'pitch':0, 'roll':0}
        self.pos = {'x':0, 'y':0}
        self.abs_pos = {'x':0, 'y':0}
        self.pointsCollected = []
        self.finalPoints = []
        self.finalDestination = []


        #Signals for state transition

        self.aborted = False
        self.killed = False
        self.completed = False


    #Python Methods
            
    def grouper(n, iterable, fillvalue=None):
        "grouper(3, 'ABCDEFG', 'x') --> ABC DEF Gxx"
        args = [iter(iterable)] * n
        return izip_longest(fillvalue=fillvalue, *args)

    #ROS Callbacks

    def handleSrv(self, req):
        if req.start_request:
            self.aborted = False
            self.depth_setpoint = req.start_ctrl.depth_setpoint
            self.active = True
        elif req.abort_request:
            self.aborted = True
        return mission_to_visionResponse(True, False)

    def navCallback(self,res):
        rospy.loginfo("Sent x and y coordinate for navigation")

    def compass_callback(self,data):
        self.heading['yaw'] = data.yaw

    def depth_callback(self,data):
        self.depth = data.depth 

    def pos_callback(self,data):
        self.pos['x'] = data.pose.pose.position.x
        self.pos['y'] = data.pose.pose.position.y

    def DVL_callback(self, data):
        self.abs_pos['x'] = data.pose.pose.x
        self.abs_pos['y'] = data.pose.pose.y
    #ROS Utility functions

    def calcAngle(self, pinger):
        #Stop robot first
        self.stopRobot() 
        argWithPinger = (self.pos['y'] - pinger[1])/(self.pos['x'] - pinger[0])
        return math.floor(np.rad2deg(np.arctan(argWithPinger)))

    def logger(self, data, pinger, filename):
        tempList = []
        with open(filename,"a") as f:
            f.write(time.strftime("%c"))
            while len(tempList) < 20:
                tempList.append([self.calcAngle(pinger), data])
                sidemove = -self.boundary if self.data < 0 else self.boundary
                self.navClient(self.pos['x'], self.pos['y'] + sidemove)              
            for item in tempList:
                f.write(str(item[0]) + "\t\t\t\t\t" + str(item[1]) + "\n")

    def sendMovement(self, f=0.0, h=None, sm=0.0, d=None):
        goal = ControllerGoal(forward_setpoint=f, heading_setpoint=h, sidemove_setpoint=sm, depth_setpoint=d)
        self.locomotionClient.send_goal(goal)
        self.locomotionClient.wait_for_result(rospy.Duration(0.5))

    def stopRobot(self):
        self.sendMovement(f=0, sm=0)

    def startSrv_Sub(self):
    #Initialise Locomotion Client 
        try:
            self.locomotionClient = actionlib.SimpleActionClient('LocomotionServer',ControllerAction) 

        except rospy.ServiceException:
            rospy.logerr("Error running Locomotion Client")
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

        
    def unregister(self):
        self.position_sub.unregister()
        self.depth_sub.unregister()
        self.compass_sub.unregister()

    def quitProgram(self, signal, frame):
        self.killed = True
        self.aborted = True

    def abortMission(self):
        if not self.test or not self.active:
            self.toMission(fail_request=True, task_complete_request=False)
        self.unregister()
        self.stopRobot()
        self.aborted = True
        self.killed = True

    def taskComplete(self):
        if not self.test and self.active:
            self.toMission(task_complete_request=True)
        self.stopRobot()
        self.aborted = True
        self.killed = True
        self.completed = True

    #Mathematical Methods 

    def normaliseHeading(self, angle, heading):
        return (DEGREE_TWO_PI+heading+angle) % (DEGREE_TWO_PI)

    def withinRange(self):
        return angle >= -self.boundary and angle <= self.boundary 

    def PassiveReading(self, conn):
        '''
        while len(data) is not self.takes:
            data = ag.music_algo(conn.recv(BUFFER_SIZE))  
            data.append(data)
        '''
        avgAngle = music_algo(conn.recv(BUFFER_SIZE))
        rospy.loginfo("Waiting for request from mission planner")
        if self.test and self.withinRange(avgAngle):
            self.pointsCollected.append(Point(self.pos['x'], self.pos['y'], self.heading['yaw'], avgAngle))

    def getEstimated(self, listOfPoints):
        if len(listOfPoints) % 2 is not 0:
            listOfPoints.pop()
        collectionOfXY = [self.triangulate(i,j) for i,j in grouper(2, listOfPoints)]
        avgX = sum([i[0] for i in collectionOfXY])/len(collectionOfXY)
        avgY = sum([i[1] for i in collectionOfXY])/len(collectionOfXY)
        return [avgX, avgY]

    def getBetterAngle(self, angle, heading):
            #Turn close to pinger
            self.sendMovement(h = angle)

    
    def getFinalPoints(self, conn):
        while len(self.data) is not self.takes:
            self.data.append(conn.recv(BUFFER_SIZE))
        avgAngle = music_algo(self.data)
        self.data = []

        while len(self.finalPoints) is not 2:
            if not withinRange(avgAngle):
                self.getBetterAngle(avgAngle, self.heading['yaw'])
                self.finalPoints.append(Point(self.pos['x'], self.pos['y'], self.heading['yaw'], avgAngle))
            else:
                self.finalPoints.append(Point(self.pos['x'], self.pos['y'], self.heading['yaw'], avgAngle))
                sidemove = -self.boundary if self.data < 0 else self.boundary
                self.navClient(self.pos['x'], self.pos['y'] + sidemove)              
        return self.triangulate(self.finalPoints[0], self.finalPoints[1])

    def triangulate(self, point1, point2):
        #X,Y Corrdinates swapped
        #return a list of xy coordinates
        heading1 = (DEGREE_TWO_PI+point1.heading+point1.angFromPing) % (DEGREE_TWO_PI)
        heading2 = (DEGREE_TWO_PI+point2.heading+point2.angFromPing) % (DEGREE_TWO_PI)
        if (((math.sin(heading1/DEGREE_PI*math.pi)) != 0) and ((math.sin(heading2/DEGREE_PI*math.pi)) != 0)):	
            m1 = (math.sin(heading1/DEGREE_PI*math.pi))	/ (math.cos(heading1/DEGREE_PI*math.pi)) 
            c1 = point1.pos['y'] - (m1*point1.pos['x'])
            m2 =   (math.sin(heading2/DEGREE_PI*math.pi)) / (math.cos(heading2/DEGREE_PI*math.pi))
            c2 = point2.pos['y'] - (m2*point2.pos['x'])
            constant_matrix    = np.matrix([[c1],[c2]])
            coefficient_matrix = np.matrix([[1,m1],[1,m2]])
            result_matrix 	   = coefficient_matrix.I * constant_matrix
            return [result_matrix[0,0],result_matrix[1,0]]                                      
        else:
            print "Division by zero has occured"                                  

    def music_algo(self, ls):
        r_cov = 0
        def splitMsg(data):
            ls = []
            (real0, imag0,
             real1, imag1,
             real2, imag2,
             real3, imag3) = data.split(',')
            hydro0_complex = np.complex(float(real0), float(imag0))
            hydro1_complex = np.complex(float(real1), float(imag1))
            hydro2_complex = np.complex(float(real2), float(imag2))
            hydro3_complex = np.complex(float(real3), float(imag3))
            ls.append(hydro0_complex)
            ls.append(hydro1_complex)
            ls.append(hydro2_complex)
            ls.append(hydro3_complex)
            return ls

        thetaM = [-45, 45, 135, 225]
        v = 1500
        f = 28000.0
        lamda = v / f
        d = 0.015
        a = math.sqrt(2 * math.pow(d / 2, 2))
        A = zeros((4, 360), dtype=complex)
        data_ls = []
        for i in len(data_ls):
            splitted = splitMsg(ls[i])
            pmusic = zeros((360))
            for theta in range(1, 361):
                for i in range(0, 4):
                    pd = a * math.cos((theta - thetaM[i]) / 180.0 * (math.pi))
                    A[i, theta - 1] = cmath.exp(2 * (1j) * math.pi * pd / lamda)
            A = np.matrix(A)
            R = np.matrix([[splitted[0]], [splitted[1]],
                           [splitted[2]], [splitted[3]]])
            cov = R * R.T.conj()
            r_cov += cov

        (eigval, eigvec) = LA.eigh(r_cov)
        Vn = eigvec[:, 0:3]
        for phi in range(1, 361):
            Ahat = A[:, phi - 1]
            num = Ahat.T.conj() * Ahat
            denom = (Ahat.T.conj() * Vn) * (Vn.T.conj() * Ahat)
            pmusic[phi - 1] = num.real / denom.real

        # print max(pmusic)
        DOA = np.argmax(pmusic)

        return DOA
        '''
        count = 0
        for i in pmusic:
            if i == max(pmusic):
                break
            count = count + 1
        DOA = count + 1
        '''
if __name__ == "__main__":
    rospy.init_node("acoustic_core")
    acousticCore =  AcousticNode()
    rospy.spin()




