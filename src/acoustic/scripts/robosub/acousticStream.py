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
from bbauv_msgs.msg import *

serverPort =5011
BUFFER_SIZE = 5021
DEGREE_PI = 180
DEGREE_TWO_PI = 360

class AcousticStream:
    def __init__(self):

        self.socket = None
        self.step_size = 4.0
        self.depth_default = 1.0
        self.duration = 1
        self.sampleAmt = 1 
        self.frequency = 30000.0
        self.bound = {'low':140, 'high':240}
          
        self.arr = zeros((360,90), dtype=object)
        self.doa = 0
        self.elevation = 0

        self.acoustic_pub = rospy.Publisher("/acoustic/pingu", pingu)

    def splitMsg(self, data):
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

    def computeCovarianceMatrix(self, complexList):
        r_conv = zeros((4, 4), dtype=complex)
        r_conv = np.matrix(r_conv)
        for T in range(self.sampleAmt):
            R = np.matrix([[(complexList[T])[0]], [(complexList[T])[1]],[(complexList[T])[2]], [(complexList[T])[3]]])
            cov = R * R.T.conj()
            r_conv += cov
        r_conv = r_conv/(self.sampleAmt)
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

    def music_3d(self, complexList):
        gamma = [-45, 45, 135, 225]
        v = 1500
        lamda = v / self.frequency
        d = 0.015
        r = math.sqrt(2 * math.pow(d / 2, 2))
        A = zeros((4, 1), dtype=complex)
        pmusic = zeros((360, 90))
        (eigval, eigvec) = LA.eigh(self.computeCovarianceMatrix(complexList))
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
        
        [Music_phiCap,Music_thetaCap] = self.getMax(pmusic)
        rospy.loginfo("Music DOA calculated: " + str(Music_phiCap))    
        rospy.loginfo("Music elevation calculated: " + str(Music_thetaCap))	
        return [Music_phiCap,Music_thetaCap]

    def generateVec(self):
        gamma = [-45, 45, 135, 225]
        v = 1500
        d = 0.015
        r = math.sqrt(2 * math.pow(d / 2, 2))
        lamda = v / self.frequency
        for theta in xrange(90):		#Theta is altitude
            for phi in xrange(360):	#Phi is azimuth
                A = zeros((4, 1), dtype=complex)
                for i in xrange(4):	#Hydrophone positions
                    pd = 2*math.pi/lamda*r*math.sin(np.deg2rad(theta))*math.cos(np.deg2rad(phi - gamma[i]))
                    A[i] = cmath.exp((1j)*pd)
                self.arr[phi, theta] = A


    def music(self,complexList):
        pmusic = zeros((360, 90))
        (eigval, eigvec) = LA.eigh(self.computeCovarianceMatrix(complexList))
        Vn = eigvec[:, 0:3]
        for theta in range(90):		#Theta is altitude
            for phi in range(360):	#Phi is azimuth
                Ahat = np.matrix(self.arr[phi, theta])
                num = Ahat.T.conj() * Ahat
                denom = (Ahat.T.conj() * Vn) * (Vn.T.conj() * Ahat)
                pmusic[phi, theta] = num.real / denom.real

        [Music_phiCap,Music_thetaCap] = self.getMax(pmusic)
        rospy.loginfo("Music DOA calculated: " + str(Music_phiCap))    
        rospy.loginfo("Music elevation calculated: " + str(Music_thetaCap))	
        return [Music_phiCap,Music_thetaCap]
        

    def stream(self):
        self.socket = socket(AF_INET, SOCK_DGRAM) 
        self.socket.bind(('192.168.1.130', serverPort))
        rospy.loginfo("Ready to go") 
        self.generateVec()
        rospy.loginfo("Steering vector generated") 
        while True:
                complexList = []
                while len(complexList) is not self.sampleAmt:
                    message, clientAddress = self.socket.recvfrom(BUFFER_SIZE)
                    rospy.loginfo(message)
                    splitted = self.splitMsg(message)
                    if not splitted: 
                        continue
                    else:
                        rospy.loginfo("Number of Ping: %d" % (len(complexList)+1))
                        complexList.append(splitted)
                self.doa, self.elevation = self.music(complexList)
                self.acoustic_pub.publish(float(self.doa), self.elevation)
                #music_3d(complexList)
                rospy.loginfo("Next")
        

if __name__ == "__main__":
    rospy.init_node("acPingu")
    Stream = AcousticStream()
    Stream.stream()

