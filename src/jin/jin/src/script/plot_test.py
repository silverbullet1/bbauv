#!/usr/bin/env python
#Get multiple datas 

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

DEGREE_PI = 180
DEGREE_TWO_PI = 360
angFromPing = []
TCP_IP = '192.168.1.141'
PORT = 5100

BUFFER_SIZE = 100  # Normally 1024, but we want fast response
data = "NO MESSAGE"

DOA = 0

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, PORT))
s.listen(1)
(conn, addr) = s.accept()

#Global

sampleAmount = sys.argv[1]

def splitTCPMsg(data):

    (real0, imag0,
     real1, imag1,
     real2, imag2,
     real3, imag3) = data.split(',')

    hydro0_complex = np.complex(float(real0), float(imag0))
    hydro1_complex = np.complex(float(real1), float(imag1))
    hydro2_complex = np.complex(float(real2), float(imag2))
    hydro3_complex = np.complex(float(real3), float(imag3))
    
    return [hydro0_complex,hydro1_complex, hydro2_complex, hydro3_complex]


def computeCovarianceMatrix(complexList):
	r_conv = zeros((4, 4), dtype=complex)
	r_conv = np.matrix(r_conv)
	for T in range(int(sampleAmount)):
		R = np.matrix([[(complexList[T])[0]], [(complexList[T])[1]],[(complexList[T])[2]], [(complexList[T])[3]]])
		cov = R * R.T.conj()
		r_conv += cov
	r_conv = r_conv/int(sampleAmount)
	return r_conv

def getMax(arrayList):
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


def classical_3d(complexList):
	max_val = 0
	phiCap = 0
	thetaCap = 0
	S = computeCovarianceMatrix(complexList)
	gamma = [-45, 45, 135, 225]
	v = 1500
	f = 30000.0
	lamda = v / f
	d = 0.015
	r = math.sqrt(2 * math.pow(d / 2, 2))
	A = zeros((4, 1), dtype=complex)
	classical = zeros((360, 90))
	for theta in range(90):		#Theta is altitude
		for phi in range(360):	#Phi is azimuth
			for i in range(4):	#Hydrophone positions
				pd = 2*math.pi/lamda*r*math.sin(np.deg2rad(theta))*math.cos(np.deg2rad(phi - gamma[i]))
				A[i] = cmath.exp((1j)*pd)
		
			Ahat = np.matrix(A)
		aTSa=(Ahat.T.conj())*S*Ahat
		classical[phi,theta] = aTSa.real
		#plot(classical[:,theta],theta)
		#writeToFile('classical.txt',classical[:,theta],theta)
	[phiCap, thetaCap] = getMax(classical)
	print ("Classical DOA calculated: " + str(phiCap))
	print ("Classical elevation calculated: " + str(thetaCap))	
	return [phiCap,thetaCap]

def music_3d(complexList):
	gamma = [-45, 45, 135, 225]
	v = 1500
	f = 30000.0
	lamda = v / f
	d = 0.015
	r = math.sqrt(2 * math.pow(d / 2, 2))
	A = zeros((4, 1), dtype=complex)
	pmusic = zeros((360, 90))
	
	(eigval, eigvec) = LA.eigh(computeCovarianceMatrix(complexList))
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
	
	[Music_phiCap,Music_thetaCap] = getMax(pmusic)
	print ("Music DOA calculated: " + str(Music_phiCap))    
	print ("Music elevation calculated: " + str(Music_thetaCap))	
	return [Music_phiCap,Music_thetaCap]

def getRawData(conn):
    complexList = []
    #sleepAwhile(10)		#Wait for certain interval before receive new data
    conn.close()
    (conn, addr) = s.accept()
    while True:
        if len(complexList) == int(sampleAmount):
            break
        else:
            data = conn.recv(BUFFER_SIZE)
            if data:
                complexList.append(splitTCPMsg(data))
                conn.close()
                (conn, addr) = s.accept()
                print("Number of takes: " + str(len(complexList)))
            else:
                print("ERROR: TCP gave wrong data")
    return complexList

def sleepAwhile(durationSec=5):
	time.sleep(durationSec)

if __name__ == "__main__":
    	rospy.init_node("ac")
	pub = rospy.Publisher("/acoustic/angFromPing", acoustic)
	while True:	
	    dat = getRawData(conn)
	    [DOA, elevationAngle] = music_3d(dat)
	    [DOA_classic, elevationAngle_classic] = classical_3d(dat)
    	conn.close()
    	rospy.spin()
