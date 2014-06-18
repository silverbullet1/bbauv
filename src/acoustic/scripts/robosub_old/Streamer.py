#!/usr/bin/env python
'''
1. Use UDP to get 2 sets of complex numbers for both pingers 
2. Use dynamic reconfigure server to determine constants
3. publish DOA & Elevation 
4. sendMovement() -> relative or absolute
5. Need to figure out a way to identify inactive pinger
'''
import rospy
import roslib;
import socket 
from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from std_msgs.msg import *
import signal 
import matplotlib.pyplot as plt
from datetime import datetime
import numpy as np
from numpy import *
from numpy import linalg as LA
from numpy import matrix 
import math
import cmath
import sys 


#Constants 
IP_ADDR = "192.168.1.203"
PORT_NUM = "5011"
BUFFER_SIZE = 360
DEGREE_PI = 180
DEGREE_TWO_PI = 360
iter = 0


    

class AcousticNode():

    def __init__(self, param):
        rospy.loginfo("Node activated")

       	#From Dynamic Server
        self.f = 37500.0
        self.step_size = 1.0
        self.elevationBound = 30
        self.sampleAmt = 4
        self.isLogged = False

        #New constants
        self.initTime = rospy.get_time()
        self.active = 0
        self.bad = 0
        self.limit = 5
        self.pingerDistance = 0
        self.protocol = rospy.get_param("~pro", "udp")
        

        #Legacy constants
        self.depth = 0
        self.heading = None
        self.isKilled = False
        self.isDormant = True
        self.inTheBox = False
        self.isTest = param
        self.counter = 0

        if(self.protocol == "udp"):
            self.socket = socket.socket(AF_INET,socket.SOCK_DGRAM)
            self.socket.bind(IP_ADDR, PORT_NUM) 
        else:
            self.socket_tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket_tcp.bind(IP_ADDR, PORT_NUM)
            self.socket_tcp.listen(1)
            
        rospy.loginfo("Connection established with sBRIO")
        self.publisher = rospy.Publisher("/acoustic/rawPing", pingData)
       


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

    def checkPing(self, input):
			return ("NaN" in input or "Inf" in input)	
    
    def isOverTime(self):
    	    return rospy.get_time() - self.initTime > 240.0
 
    def getRawDataTCP(self):
        #Store array of complex number
        complexList = []
        (conn, addr) = self.socket_tcp.accept()
        while True:
            conn.close()
            (conn, addr) = self.socket_tcp.accept()
            data = conn.recv(BUFFER_SIZE)
            if (len(complexList) == self.sampleAmt):
            	break
            else:
                splitted = self.splitMsg(data)
             	if(self.bad > self.limit or isOverTime()):
             		self.isKilled = true	
             		break
            	elif(splitted):
            		if(self.checkPing(splitted)):
            			complexList.append(splitted)
            		else:
            		    rospy.loginfo("Bad ping detected")
            		    self.bad += 1
				        #Reverse to reduce power intensity
            		    self.sendMovement(-2.0)
            		    continue
            	else:
            	  rospy.loginfo("Value Error from data provided")
            	  continue
        #Try using every data provided from sbRIO
        return complexList

    def getRawData(self):
        #Store array of complex number
        complexList = []
        while True:
            (data, addr) = self.socket.recvfrom(BUFFER_SIZE)
            if (len(complexList) == self.sampleAmt):
            	break
            else:
                splitted = self.splitMsg(data)
             	if(self.bad > self.limit or isOverTime()):
             		self.isKilled = true	
             		break
            	elif(splitted):
            		if(self.checkPing(splitted)):
            			complexList.append(splitted)
            		else:
            		    rospy.loginfo("Bad ping detected")
            		    self.bad += 1
				        #Reverse to reduce power intensity
            		    self.sendMovement(-2.0)
            		    continue
            	else:
            	  rospy.loginfo("Value Error from data provided")
            	  continue
        #Try using every data provided from sbRIO
        return complexList

    def splitMsg(self, data):
        try:
            (real0, imag0,
             real1, imag1,
             real2, imag2,
             real3, imag3) = data.split(',')
        except ValueError:
            return False
       
	cmp0 = np.complex(float(real0), float(imag0))
	cmp1 = np.complex(float(real1), float(imag1))
	cmp2 = np.complex(float(real2), float(imag2))
	cmp3 = np.complex(float(real3), float(imag3))
 
	return [cmp0, cmp1, cmp2, cmp3]

    def computeCovarianceMatrix(self, complexList):
        r_conv = zeros((4, 4), dtype=complex)
        r_conv = np.matrix(r_conv)
        for T in range(self.sampleAmt):
            R = np.matrix([[(complexList[T])[0]], [(complexList[T])[1]],[(complexList[T])[2]], [(complexList[T])[3]]])
            cov = R * R.T.conj()
            r_conv += cov
        r_conv = r_conv/(self.sampleAmt)
        return r_conv

    def music_3d(self,complexList):
        #Frequency, f in Hertz
        gamma = [-45, 45, 135, 225]
        v = 1500
        lamda = v / self.f
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
        if self.isLogged:
       	    plot(pmusic[:,theta],theta)
            writeToFile('pmusic.txt',pmusic[:,theta],theta)
        
        [Music_phiCap,Music_thetaCap] = self.getMax(pmusic)
        rospy.loginfo("Music DOA calculated: " + str(Music_phiCap))    
        rospy.loginfo("Music elevation calculated: " + str(Music_thetaCap))	
        return [Music_phiCap,Music_thetaCap]

    def connect(self):

        while not rospy.is_shutdown():
            global iter
            if(self.protocol == "udp"):
                tmp = self.getRawData()
            else:
                tmp = self.getRawDataTCP()

            (DOA, Elevation) = self.music3d(tmp)
            self.publisher.publish(DOA, Elevation, iter+1)
        
    def terminate(self, signum, frame):
        self.socket.close()

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

    def overShotPinger(self, angle):
        return 120<= angle <=240

    def initAll(self):
    #Initialise Locomotion Client 
        try:
            self.locomotionClient = actionlib.SimpleActionClient('LocomotionServer',ControllerAction) 
            self.locomotionClient.wait_for_server(timeout=rospy.Duration(5))
        except rospy.ServiceException:
            rospy.logerr("Error running Locmotion Client")
    #Initialise Controller Service:
        try:
            self.controllerSettings = rospy.ServiceProxy("/set_controller_srv", set_controller)
            self.controllerSettings.wait_for_service()
        except rospy.ServiceException:
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
        
    #Utility for logging
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

        	    
if __name__ == "__main__":
    rospy.init_node("/acoustic/Streamer")
    st = Streamer()
    signal.signal(signal.SIGINT, st.terminate())
    signal.signal(signal.SIGTSTP, st.terminate())
    try:
        st.connect()
    except rospy.ROSInterruptException:
        rospy.logerr("Interrupt Exception")
    


