#!/usr/bin/env python

import matplotlib.pyplot as plt
from datetime import datetime
import time
import numpy as np
import rospy
import roslib
from numpy import linalg as LA
from numpy import *
from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from numpy import matrix 
import math
import cmath
import array
import socket 
import sys
from std_msgs.msg._Float32 import Float32
import signal
import actionlib
import signal

DEGREE_PI = 180
DEGREE_TWO_PI = 360
angFromPing = []
TCP_IP = '192.168.1.141'
PORT = 5100

BUFFER_SIZE = 100  # Normally 1024, but we want fast response
data = "NO MESSAGE"

s= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, PORT))
s.listen(1)
(conn, addr) = s.accept()

#sampleAmount = sys.argv[1]
sampleAmount = 1

class Plot:
    
    def __init__(self):
        #For state machine
        self.numberData = 0
        self.DOA = 0
        self.Elevation = 100
        
        self.DOA_classical = 0
        self.DOA_music = 0
        self.Elevation_classical = 0
        self.Elevation_music = 0
        
        self.heading = 0
        self.pingDetected = False
        self.calculatingData = False
        signal.signal(signal.SIGINT, self.userQuit)
        
        self.locomotionClient = actionlib.SimpleActionClient('LocomotionServer',ControllerAction) 
        setServer = rospy.ServiceProxy("/set_controller_srv", set_controller)
        setServer(forward=True, sidemove=True, heading=True, depth=True, pitch=True, 
                    roll=False, topside=False, navigation=False)
        rospy.loginfo("Set server already")
                 
        try:
             self.locomotionClient.wait_for_server(timeout=rospy.Duration(5))
        except Exception as e: 
             rospy.loginfo("Error connecting to locomotion server")

        
        compass_sub = rospy.Subscriber('/euler', bbauv_msgs.msg.compass_data, self.compass_callback)
        
    def userQuit(self, signal, frame):
        rospy.signal_shutdown("Bye!")
    
    def compass_callback(self, data):
        self.heading = data.yaw
        
        (conn, addr) = s.accept()
        dat = self.getRawData(conn)
        rospy.loginfo("Calculating data")
        
        #dat = [0.9389-3.6210j, -2.606701.7737j,0.4586+2.1283j,-1.9776+2.6057j]
        
        complexList = self.computeCovarianceMatrix(dat)

        #[self.DOA_classical, self.Elevation_classical] = self.classical_3d(complexList)
        [self.DOA_music, self.Elevation_music] = self.music_3d(complexList)
        #(conn, addr) = s.accept()
        self.calculatingData = False

    def splitTCPMsg(self, data):
        (real0, imag0,
         real1, imag1,
         real2, imag2,
         real3, imag3) = data.split(',')
    
        hydro0_complex = np.complex(float(real0), float(imag0))
        hydro1_complex = np.complex(float(real1), float(imag1))
        hydro2_complex = np.complex(float(real2), float(imag2))
        hydro3_complex = np.complex(float(real3), float(imag3))
        
        return [hydro0_complex,hydro1_complex, hydro2_complex, hydro3_complex]


    def computeCovarianceMatrix(self, complexList):
        r_conv = zeros((4, 4), dtype=complex)
        r_conv = np.matrix(r_conv)
        for T in range(int(sampleAmount)):
            R = np.matrix([[(complexList[T])[0]], [(complexList[T])[1]],[(complexList[T])[2]], [(complexList[T])[3]]])
            cov = R * R.T.conj()
            r_conv += cov
        r_conv = r_conv/int(sampleAmount)
        return r_conv
    
    def getMax(self, arrayList):
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


    def classical_3d(self, covarianceMatrix):
        rospy.loginfo("Classical")
        max_val = 0
        phiCap = 0
        thetaCap = 0
        S = covarianceMatrix
        
        gamma = [-45, 45, 135, 225]
        v = 1500
        f = 30000.0
        lamda = v / f
        d = 0.015
        r = math.sqrt(2 * math.pow(d / 2, 2))
        A = zeros((4, 1), dtype=complex)
        classical = zeros((360, 90))
        for theta in range(90):        #Theta is altitude
            for phi in range(360):    #Phi is azimuth
                for i in range(4):    #Hydrophone positions
                    pd = 2*math.pi/lamda*r*math.sin(np.deg2rad(theta))*math.cos(np.deg2rad(phi - gamma[i]))
                    A[i] = cmath.exp((1j)*pd)
            
                Ahat = np.matrix(A)
            aTSa=(Ahat.T.conj())*S*Ahat
            classical[phi,theta] = aTSa.real
            #plot(classical[:,theta],theta)
            #writeToFile('classical.txt',classical[:,theta],theta)
        [phiCap, thetaCap] = self.getMax(classical)
        print ("Classical DOA calculated: " + str(phiCap))
        print ("Classical elevation calculated: " + str(thetaCap)) 
        
        self.DOA_classical = phiCap
        self.Elevation_classical = thetaCap
        self.pingDetected = True
           
        return [phiCap,thetaCap]

    def music_3d(self, covarianceMatrix):
        rospy.loginfo("Music")
        #print covarianceMatrix
        gamma = [-45, 45, 135, 225]
        v = 1500
        f = 30000.0
        lamda = v / f
        d = 0.015
        r = math.sqrt(2 * math.pow(d / 2, 2))
        A = zeros((4, 1), dtype=complex)
        pmusic = zeros((360, 90))
        
        (eigval, eigvec) = LA.eigh(covarianceMatrix)
        print "EigVal"
#         (eigval, eigvec) = LA.eigh(self.computeCovarianceMatrix(complexList))
        Vn = eigvec[:, 0:3]
        
        print "going loop"
        for theta in range(90):        #Theta is altitude
            for phi in range(360):    #Phi is azimuth
                for i in range(4):    #Hydrophone positions
                    pd = 2*math.pi/lamda*r*math.sin(np.deg2rad(theta))*math.cos(np.deg2rad(phi - gamma[i]))
                    A[i] = cmath.exp((1j)*pd)
            
                Ahat = np.matrix(A)
                num = Ahat.T.conj() * Ahat
                denom = (Ahat.T.conj() * Vn) * (Vn.T.conj() * Ahat)
                pmusic[phi, theta] = num.real / denom.real
        print "theta"
        
        [Music_phiCap,Music_thetaCap] = self.getMax(pmusic)
        print ("Music DOA calculated: " + str(Music_phiCap))    
        print ("Music elevation calculated: " + str(Music_thetaCap))  

        self.DOA_music = Music_phiCap
        self.Elevation_music = Music_thetaCap
        self.pingDetected = True

        return [Music_phiCap,Music_thetaCap]
    
    def getRawData(self, conn):
        complexList = []
        #sleepAwhile(10)        #Wait for certain interval before receive new data
        conn.close()
        (conn, addr) = s.accept()
        while True:
            if len(complexList) == int(sampleAmount):
                break
            else:
                data = conn.recv(70)
                if data:
                    complexList.append(self.splitTCPMsg(data))
                    conn.close()
                    (conn, addr) = s.accept()
                    print("Number of takes: " + str(len(complexList)))
                else:
                    print("ERROR: TCP gave wrong data")
        return complexList
    
    def sendMovement(self, forward=0.0, turn=None, depth=0.6):
        rospy.loginfo("Heading: {} Turn:{}".format(self.heading, turn))
        turn = ((turn+self.heading)%360)-7 if turn else self.heading
        goal = ControllerGoal(forward_setpoint=forward, heading_setpoint=turn, 
                              sidemove_setpoint=0.0, depth_setpoint=depth)
        rospy.loginfo("Forward: {} heading: {}".format(forward, turn))
        self.locomotionClient.send_goal(goal)
        self.locomotionClient.wait_for_result()
        #self.locomotionClient.wait_for_result(rospy.Duration(0.5))
        rospy.loginfo("Completed")
    
if __name__ == "__main__":
    rospy.init_node("ac_lynnette")
    pub = rospy.Publisher("/acoustic/angFromPing", acoustic)
    plot = Plot()
#     dat = plot.getRawData(conn)
#     complexList = plot.computeCovarianceMatrix(dat)

#     [plot.DOA_classical, plot.Elevation_classical] = plot.classical_3d(complexList)
#     [plot.DOA_classical, plot.Elevation_classical] = plot.classical_3d(complexList)
#     [plot.DOA_music, plot.Elevation_music] = plot.music_3d(dat)
    rospy.spin()
    
    
    
    
    
    