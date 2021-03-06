#!/usr/bin/env python
#Get multiple datas 

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

DEGREE_PI = 180
DEGREE_TWO_PI = 360
angFromPing = []
TCP_IP = '192.168.1.149'
PORT = 5100

BUFFER_SIZE = 100  # Normally 1024, but we want fast response
data = "NO MESSAGE"

DOA = 89

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, PORT))
s.listen(1)

#Global

take = sys.argv[1]
thetaM = [-45, 45, 135, 225]
v = 1500
f = 28000.0
lamda = v / f
d = 0.015
a = math.sqrt(2 * math.pow(d / 2, 2))
A = zeros((4, 360), dtype=complex)

def splitMsg(data):

    (real0, imag0,
     real1, imag1,
     real2, imag2,
     real3, imag3) = data.split(',')

    hydro0_complex = np.complex(float(real0), float(imag0))
    hydro1_complex = np.complex(float(real1), float(imag1))
    hydro2_complex = np.complex(float(real2), float(imag2))
    hydro3_complex = np.complex(float(real3), float(imag3))
    
    return [hydro0_complex,hydro1_complex, hydro2_complex, hydro3_complex]

def music_algo(ls):
    pub_pmusic = rospy.Publisher("/acoustic/pmusic", Float32)
    global DOA, A
    r_conv = 0
    for x in range(int(take)):
        pmusic = zeros((360))
        for theta in range(1, 361):
            for i in range(0, 4):
                pd = a * math.cos((theta - thetaM[i]) / 180.0 * (math.pi))
                A[i, theta - 1] = cmath.exp(2 * (1j) * math.pi * pd / lamda)
        A = np.matrix(A)
        #print ls[x]
        R = np.matrix([[(ls[x])[0]], [(ls[x])[1]],
                       [(ls[x])[2]], [(ls[x])[3]]])
        cov = R * R.T.conj()
        #print cov
        r_conv += cov
    r_conv = r_conv/int(take)
    #print r_conv
    (eigval, eigvec) = LA.eigh(r_conv)
    Vn = eigvec[:, 0:3]
    for phi in range(1, 361):
        Ahat = A[:, phi - 1]
        num = Ahat.T.conj() * Ahat
        denom = (Ahat.T.conj() * Vn) * (Vn.T.conj() * Ahat)
        pmusic[phi - 1] = num.real / denom.real

    # print max(pmusic)
    #DOA = np.argmax(pmusic)
    with open('pmusic.txt', 'a') as pfile:
        for pm in pmusic:
            pfile.write(pm)
            pfile.write("\n")

    count = 0
    for i in pmusic:
        if i == max(pmusic):
            break
        count = count + 1
    DOA = count + 1

def triangulate(doa1, doa2):

    h1 = 60
    h2 = 80
    x1_pos = 6 
    y1_pos = 12
    x2_pos = 6 
    y2_pos = 4
    #X,Y Corrdinates swapped
    #return a list of xy coordinates
    heading1 = (DEGREE_TWO_PI+h1+doa1) % (DEGREE_TWO_PI)
    heading2 = (DEGREE_TWO_PI+h2+doa2) % (DEGREE_TWO_PI)
    if (((math.sin(heading1/DEGREE_PI*math.pi)) != 0) and ((math.sin(heading2/DEGREE_PI*math.pi)) != 0)):	
        m1 = (math.sin(heading1/DEGREE_PI*math.pi))	/ (math.cos(heading1/DEGREE_PI*math.pi)) 
        c1 = y1_pos - (m1*x1_pos)
        m2 =   (math.sin(heading2/DEGREE_PI*math.pi)) / (math.cos(heading2/DEGREE_PI*math.pi))
        c2 = y2_pos - (m2*x2_pos)
        constant_matrix    = np.matrix([[c1],[c2]])
        coefficient_matrix = np.matrix([[1,m1],[1,m2]])
        result_matrix 	   = coefficient_matrix.I * constant_matrix
        return [result_matrix[0,0],result_matrix[1,0]]                                      
    else:
        print "Division by zero has occured"                                  
if __name__ == "__main__":
    rospy.init_node("ac")
    pub = rospy.Publisher("/acoustic/angFromPing", bbauv_msgs.msg.acoustic)
    while True:
        (conn, addr) = s.accept()
        data = conn.recv(BUFFER_SIZE)
        if('NaN' in data or 'Inf' in data):
            print "NaNster is here"
        else:
            print "Correct"
        ls = []
        while len(ls) is not int(take):
            if data:
                ls.append(splitMsg(data))
                conn.close()
                (conn, addr) = s.accept()
                data = conn.recv(BUFFER_SIZE)
            print data
        music_algo(ls)
        angFromPing.append(DOA)
        ls = []
        print "DOA is: ", DOA, " degrees"
        pub.publish(DOA)
    rospy.spin()
    conn.close()
    #print triangulate(angFromPing[0], angFromPing[1])

