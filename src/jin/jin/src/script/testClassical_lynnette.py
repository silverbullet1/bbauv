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

sampleAmount = 1

def computeCovarianceMatrix(complexList):
    r_conv = zeros((4, 4), dtype=complex)
    r_conv = np.matrix(r_conv)
    for T in range(int(sampleAmount)):
        R = np.matrix([[(complexList[T])[0]], [(complexList[T])[1]],[(complexList[T])[2]], [(complexList[T])[3]]])
        cov = R * R.T.conj()
        r_conv += cov
    r_conv = r_conv
    
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
           
    return [phiCap,thetaCap]

if __name__ == "__main__":
	rospy.init_node("ac_lynnette")
	dat = [[[0.9389-3.6210j], [-2.60670+1.7737j], [0.4586+2.1283j], [-1.9776+2.6057j]]]
	#print dat
	complexList = computeCovarianceMatrix(dat)
	print complexList 
	
	[DOA_classical, Elevation_classical] = classical_3d(complexList)
	rospy.spin()
    
    
    
    