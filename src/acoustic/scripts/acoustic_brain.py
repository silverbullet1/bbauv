#!/usr/bin/env python 
import rospy	
import roslib
import time
import matplotlib.pyplot as plt
from datetime import datetime
from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from nav_msgs.msg import Odometry
from std_msgs.msg._Float32 import Float32
import actionlib
import numpy as np
from numpy import *
from numpy import linalg as LA
from numpy import matrix 
import math
import cmath
import sys 
import socket
import signal

#Constants
DEGREE_PI = 180
DEGREE_TWO_PI = 360
TCP_IP = '192.168.1.100'
TCP_PORT = 5100
BUFFER_SIZE = 360        
sampleAmount = 2	#Global constant
distanceConst = [3.0,2.0,1.0]
step_size = 1.0

class AcousticNode(object):

    def __init__(self, param):
        rospy.loginfo("Intialised brain")
        self.depth = 0
        self.heading = None
        self.elevationAngle = 0
        self.DOA = 0
        self.DOA2 = 0
        self.pingerDistance = 0
        self.isKilled = False
        self.isDormant = True
        self.inTheBox = False
        self.isTest = param
        self.fh = 37500.0 #Higher frequency pinger
        self.fl = 22500.0 #Lower frequency pinger
        self.counter = 0
        self.bad = 0
        self.s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.s.bind((TCP_IP, TCP_PORT))
        self.TCP_connect = None
        rospy.loginfo("Connected to sbRIO")

    def handleSrv(self, req):
        if req.start_request:
            self.isDormant = False
        elif req.abort_request:
            self.isKilled = True
        return mission_to_visionResponse(start_response=True, abort_response=False, data=controller())

    def compass_callback(self,data):
        self.heading = data.yaw

    def depth_callback(self,data):
        self.depth = data.depth 

    #ROS Utility functions

    def sendMovement(self,forward=0.0, sidemove=0.0, turn=None, depth=1.0, absolute=False, wait=True):
        if turn is None:
            turn = self.heading
            goal = ControllerGoal(forward_setpoint=forward, heading_setpoint=turn, sidemove_setpoint=sidemove, depth_setpoint=depth)
        else:
            if absolute:
                goal = ControllerGoal(forward_setpoint=forward, heading_setpoint=turn, sidemove_setpoint=sidemove, depth_setpoint=depth)
            else:
                turn = (turn+self.heading)%360 
                goal = ControllerGoal(forward_setpoint=forward, heading_setpoint=turn, sidemove_setpoint=sidemove, depth_setpoint=depth)
        rospy.loginfo("Turn received: " + str(turn))
        self.locomotionClient.send_goal(goal)
        if wait:
            self.locomotionClient.wait_for_result()
        else:
            self.locomotionClient.wait_for_result(timeout=rospy.Duration(2.0))


    def initAll(self):
    #Initialise Locomotion Client 
        try:
            self.locomotionClient = actionlib.SimpleActionClient('LocomotionServer',ControllerAction) 
            self.locomotionClient.wait_for_server(timeout=rospy.Duration(5))
        except rospy.ServiceException:
            self.TCP_connect.shutdown()
            self.TCP_connect.close()
            rospy.logerr("Error running Locmotion Client")
    #Initialise Controller Service:
        try:
            self.controllerSettings = rospy.ServiceProxy("/set_controller_srv", set_controller)
            self.controllerSettings.wait_for_service()
        except rospy.ServiceException:
            self.TCP_connect.shutdown()
            self.TCP_connect.close()
            rospy.logerr("Failed to connect to Controller")


    #Initialise acoustic to mission 
        try:
            self.toMission = rospy.ServiceProxy("/acoustic/vision_to_mission", vision_to_mission)
            self.toMission.wait_for_service(timeout=60)
        except rospy.ServiceException:
            rospy.logerr("Error connecting to mission planner")

    #Setting controller server
        setServer = rospy.ServiceProxy("/set_controller_srv", set_controller)
        #setServer(forward=True, sidemove=True, heading=True, depth=False, pitch=True, roll=True, topside=False, navigation=False)

    #Initialise depth_sub
        self.depth_sub = rospy.Subscriber("/depth", depth, self.depth_callback)

    #Inialise compass_sub 
        self.compass_sub = rospy.Subscriber('/euler', compass_data, self.compass_callback)
        while self.heading is None:
            rospy.loginfo("Waiting for compass")
            rospy.sleep(rospy.Duration(1.0))
        
    def unregister(self):
        self.depth_sub.unregister()
        self.compass_sub.unregister()

        #Mathematical Methods 

    def splitTCPMsg(self, data, num):
        
        try:
            (real0, imag0,
             real1, imag1,
             real2, imag2,
             real3, imag3,
             real4, imag4,
             real5, imag5,
             real6, imag6,
             real7, imag7) = data.split(',')
            a = [real0, imag0, real1, imag1, real2, imag2, real3, imag3] if num == 1 else [real4, imag4, real5, imag5, real6, imag6, real7, imag7]
        except ValueError:
            return False
            

        if("NaN" in a or "Inf" in a):
            return False
        else:
            hydro0_complex = np.complex(float(a[0]), float(a[1]))
            hydro1_complex = np.complex(float(a[2]), float(a[3]))
            hydro2_complex = np.complex(float(a[4]), float(a[5]))
            hydro3_complex = np.complex(float(a[6]), float(a[7]))
            return [hydro0_complex,hydro1_complex, hydro2_complex, hydro3_complex]

    def computeCovarianceMatrix(self, complexList):
        global sampleAmount
        r_conv = zeros((4, 4), dtype=complex)
        r_conv = np.matrix(r_conv)
        for T in range(sampleAmount-1):
            R = np.matrix([[(complexList[T])[0]], [(complexList[T])[1]],[(complexList[T])[2]], [(complexList[T])[3]]])
            cov = R * R.T.conj()
            r_conv += cov
        r_conv = r_conv/(sampleAmount-1)
        return r_conv

    def getMax(self,arrayList):
        max_val = 0
        phiCap = 0
        thetaCap = 0
        for theta in range(len(arrayList[0])):
            for phi in range(len(arrayList)):	
                if max_val < arrayList[phi, theta]:
                    max_val = arrayList[phi, theta]
                    phiCap = phi
                    thetaCap = theta
        return [phiCap,thetaCap]

    def music_3d(self,complexList, f):
        #Frequency, f in Hertz
        gamma = [-45, 45, 135, 225]
        v = 1500
        lamda = v / f
        d = 0.015
        r = math.sqrt(2 * math.pow(d / 2, 2))
        A = zeros((4, 1), dtype=complex)
        pmusic = zeros((360, 90))
        
        (eigval, eigvec) = LA.eigh(self.computeCovarianceMatrix(complexList))
        Vn = eigvec[:, 0:3]
        
        for theta in range(90):		#Theta is altitude
            for phi in range(360):	#Phi is azimuth
                for i in range(4):	#Hydrophone positions
                    pd = 2*math.pi/lamda*r*math.sin(np.deg2rad(theta))*math.cos(np.deg2rad(phi - gamma[i]))
                    A[i] = cmath.exp((1j)*pd)
            
                Ahat = np.matrix(A)
                num = Ahat.T.conj() * Ahat
                denom = (Ahat.T.conj() * Vn) * (Vn.T.conj() * Ahat)
                pmusic[phi, theta] = num.real / denom.real
                #plot(pmusic[:,theta],theta)
                #writeToFile('pmusic.txt',pmusic[:,theta],theta)
        
        [Music_phiCap,Music_thetaCap] = self.getMax(pmusic)
        rospy.loginfo("Music DOA calculated: " + str(Music_phiCap))    
        rospy.loginfo("Music elevation calculated: " + str(Music_thetaCap))	
        return [Music_phiCap,Music_thetaCap]

    def plot(self,list,Phi):
        plt.plot([i for i in range(360)], list) 
        plt.savefig(str(datetime.now())+"_"+str(Phi)+".png")	
        
    def writeToFile(self,fileName,list,theta):
        with open(fileName, 'a') as pfile:
            pfile.write("....BEGIN OF " + str(theta) + " FILE....")
            for pm in list:
                pfile.write(str(pm))
                pfile.write("\n") 
            pfile.write("....END OF " + str(theta) + " FILE....")

    def getRawData(self,conn,num):
        complexList = []
        conn.close()
        (conn, addr) = self.s.accept()
        while True:
            if len(complexList) == sampleAmount:
                break
            else:
                data = conn.recv(BUFFER_SIZE)
                rospy.loginfo(data)
                if(self.splitTCPMsg(data, num) and self.bad == 1):
                        complexList = []
                        self.bad = 0
                        rospy.loginfo("Recovered from one bad ping")
                        splitted = self.splitTCPMsg(data, num)
                        complexList.append(splitted)
                        conn.close()
                        (conn, addr) = self.s.accept()
                        break
                elif(self.splitTCPMsg(data, num)):
                        rospy.loginfo("Good ping")
                        splitted = self.splitTCPMsg(data, num)
                        complexList.append(splitted)
                        conn.close()
                        (conn, addr) = self.s.accept()
                        rospy.loginfo("Number of takes: " + str(len(complexList)))
                else:
                    rospy.loginfo("Bad ping received")
                    if self.bad > 1:
                        self.sendMovement(forward=0.0, turn=20, depth=1.0)
                        self.bad = 0
                    else:
                        self.bad += 1
                    conn.close()
                    (conn, addr) = self.s.accept()
                    continue
        return complexList[1:sampleAmount]
                    
    def sleepAwhile(self,durationSec=5):
        time.sleep(durationSec)

    def overShotPinger(self, angle):
        return 120<= angle <=240

    def calculateDOA(self, conn, num):
        global distanceConst, step_size
        final_rawData = self.getRawData(conn, num)
        if num == 1:
            [self.DOA,self.elevationAngle] = self.music_3d(final_rawData, self.fh) if len(final_rawData) is not 0 else [0,0]
        else:
            [self.DOA,self.elevationAngle] = self.music_3d(final_rawData, self.fl) if len(final_rawData) is not 0 else [0,0]
        if self.overShotPinger(self.DOA) and self.counter > 1:
            self.pingerDistance = -(self.pingerDistance/2.0)
        else:
            self.pingerDistance = distanceConst[self.counter] if self.counter < 2 else step_size
        self.counter+= 1
        rospy.loginfo("AUV is " + str(self.pingerDistance) + " m away")

    def quitProgram(self, signal, frame):
        self.TCP_connect.shutdown()
        self.TCP_connect.close()
        self.isKilled = True

if __name__ == "__main__":
    try:
        rospy.loginfo("init successful")
        rospy.init_node("acoustic_core")
        acousticCore = AcousticNode()
        signal.signal(signal.SIGINT, acousticCore.quitProgram)
        signal.signal(signal.SIGTERM, acousticCore.quitProgram)
        rospy.spin()
    except KeyboardInterrupt:
        self.TCP_connect.shutdown()
        self.TCP_connect.close()




