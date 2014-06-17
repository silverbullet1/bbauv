#!/usr/bin/python
import rospy
import roslib
from socket import *
import time
import timeit
import numpy as np
from numpy import *
from numpy import linalg as LA
from numpy import matrix 
import math
import cmath
from bbauv_msgs.msg import pingData
from comm import Comm

serverPort =5011
BUFFER_SIZE = 5021
sampleAmt = 1
DEGREE_PI = 180
DEGREE_TWO_PI = 360
frequency = 30000.0
serverSocket = socket(AF_INET, SOCK_DGRAM) 
serverSocket.bind(('192.168.1.130', serverPort))
arr = zeros((360,90), dtype=object)
pmusic_global = zeros((360,90))
DOA = 0
ELEVATION = 0
step_size = 4.0
depth_default = 1.0
count = 0
duration = 5

def splitMsg(data):
    if("NaN" in data or "Inf" in data):
        print "NaNster found"
        return False
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

def computeCovarianceMatrix(complexList):
    global sampleAmt
    r_conv = zeros((4, 4), dtype=complex)
    r_conv = np.matrix(r_conv)
    for T in range(sampleAmt):
        R = np.matrix([[(complexList[T])[0]], [(complexList[T])[1]],[(complexList[T])[2]], [(complexList[T])[3]]])
        cov = R * R.T.conj()
        r_conv += cov
    r_conv = r_conv/(sampleAmt)
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

def music_3d(complexList):
    global frequency, isLogged
    gamma = [-45, 45, 135, 225]
    v = 1500
    lamda = v / frequency
    d = 0.015
    r = math.sqrt(2 * math.pow(d / 2, 2))
    A = zeros((4, 1), dtype=complex)
    pmusic = zeros((360, 90))
    (eigval, eigvec) = LA.eigh(computeCovarianceMatrix(complexList))
    Vn = eigvec[:, 0:3]
    for theta in xrange(90):		#Theta is altitude
        for phi in xrange(360):	#Phi is azimuth
            for i in xrange(4):	#Hydrophone positions
                pd = 2*math.pi/lamda*r*math.sin(np.deg2rad(theta))*math.cos(np.deg2rad(phi - gamma[i]))
                A[i] = cmath.exp((1j)*pd)
        
            Ahat = np.matrix(A)
            num = Ahat.T.conj() * Ahat
            denom = (Ahat.T.conj() * Vn) * (Vn.T.conj() * Ahat)
            pmusic[phi, theta] = num.real / denom.real
    
    [Music_phiCap,Music_thetaCap] = getMax(pmusic)
    print "Old Music 3d"
    print("Music DOA calculated: " + str(Music_phiCap))    
    print("Music elevation calculated: " + str(Music_thetaCap))	
    return [Music_phiCap,Music_thetaCap]

def generateVec():
    global arr, frequency
    gamma = [-45, 45, 135, 225]
    v = 1500
    d = 0.015
    r = math.sqrt(2 * math.pow(d / 2, 2))
    lamda = v / frequency
    for theta in xrange(90):		#Theta is altitude
        for phi in xrange(360):	#Phi is azimuth
            A = zeros((4, 1), dtype=complex)
            for i in xrange(4):	#Hydrophone positions
                pd = 2*math.pi/lamda*r*math.sin(np.deg2rad(theta))*math.cos(np.deg2rad(phi - gamma[i]))
                A[i] = cmath.exp((1j)*pd)
            arr[phi, theta] = A


def music(complexList):
    global frequency, arr
    pmusic = zeros((360, 90))
    (eigval, eigvec) = LA.eigh(computeCovarianceMatrix(complexList))
    Vn = eigvec[:, 0:3]
    for theta in range(90):		#Theta is altitude
        for phi in range(360):	#Phi is azimuth
            Ahat = np.matrix(arr[phi, theta])
            num = Ahat.T.conj() * Ahat
            denom = (Ahat.T.conj() * Vn) * (Vn.T.conj() * Ahat)
            pmusic[phi, theta] = num.real / denom.real

    [Music_phiCap,Music_thetaCap] = getMax(pmusic)
    rospy.loginfo("Music DOA calculated: " + str(Music_phiCap))    
    rospy.loginfo("Music elevation calculated: " + str(Music_thetaCap))	
    return [Music_phiCap,Music_thetaCap]
    

def overShotPinger(angle):
    return 140 < angle < 250

def adjustStep(eleation):
    global ELEVATION, step_size
    if(60<ELEVATION<70):
        step_size = 2.0
    elif(40<ELEVATION<50):
        step_size = 1.0
        
def main():
    global ELEVATION, DOA, step_size, depth_default, count, duration
    acoustic_pub = rospy.Publisher("/acoustic/pingData", pingData)
    com = Comm()
    rospy.loginfo("Ready to go") 
    generateVec()
    rospy.loginfo("Steering vector generated") 

    while True:
        
        adjustStep(ELEVATION)    
        if ELEVATION < 30 and count > 1:
            rospy.loginfo("surfacing")
            com.sendMovement(depth=-0.1)
            break
        elif overShotPinger(DOA) and count > 1:
            rospy.loginfo("overshot")
            com.sendMovement(forward=-(step_size/2))
            com.sendMovement(depth=-0.1)
            break
            
        else:
            if count > 1:
                duration = 2
            time.sleep(duration)
            complexList = []
            while len(complexList) is not sampleAmt:
                message, clientAddress = serverSocket.recvfrom(BUFFER_SIZE)
                print message
                splitted = splitMsg(message)
                if not splitted: 
                    continue
                else:
                    print "Number of Ping: %d" % (len(complexList)+1)
                    complexList.append(splitted)
            count += 1
            DOA, ELEVATION = music(complexList)
            acoustic_pub.publish(doa, elevation)
            com.sendMovement(turn=DOA, depth=depth_default)
            com.sendMovement(forward=step_size, depth=depth_default)
            #music_3d(complexList)
            rospy.loginfo("Next")
        

if __name__ == "__main__":
    rospy.init_node("acoustic_stream")
    main()
