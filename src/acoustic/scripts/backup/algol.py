#!/usr/bin/env python
#Get multiple datas 

import numpy as np
import rospy
import roslib
from numpy import linalg as LA
from numpy import *
from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
import math
import cmath
import array
import socket
import sys

TCP_IP = '192.168.1.149'
PORT = 5100

BUFFER_SIZE = 60  # Normally 1024, but we want fast response
data = "NO MESSAGE"

r_conv = []
hydro0_complex = 0 + 0j
hydro1_complex = 0 + 0j
hydro2_complex = 0 + 0j
hydro3_complex = 0 + 0j
DOA = 0

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, PORT))
s.listen(1)
data_ls = []

#Global

thetaM = [-45, 45, 135, 225]
v = 1500
f = 28000.0
lamda = v / f
d = 0.015
a = math.sqrt(2 * math.pow(d / 2, 2))
A = zeros((4, 360), dtype=complex)

def splitMsg(data):
    global hydro0_complex, hydro1_complex, hydro2_complex, hydro3_complex
    (real0, imag0, real1, imag1, real2, imag2, real3, imag3) = data.split(',')
    hydro0_complex = np.complex(float(real0), float(imag0))
    hydro1_complex = np.complex(float(real1), float(imag1))
    hydro2_complex = np.complex(float(real2), float(imag2))
    hydro3_complex = np.complex(float(real3), float(imag3))
        

def music_algo():
    global DOA, A
    pmusic = zeros((360))
    for theta in range(1, 361):
        for i in range(0, 4):
            pd = a * math.cos((theta - thetaM[i]) / 180.0 * (math.pi))
            A[i, theta - 1] = cmath.exp(2 * (1j) * math.pi * pd / lamda)
    A = np.matrix(A)
    R = np.matrix([[hydro0_complex], [hydro1_complex],
                   [hydro2_complex], [hydro3_complex]])
    cov = R * R.T.conj()
    (eigval, eigvec) = LA.eigh(cov)
    Vn = eigvec[:, 0:3]
    for phi in range(1, 361):
        Ahat = A[:, phi - 1]
        num = Ahat.T.conj() * Ahat
        denom = (Ahat.T.conj() * Vn) * (Vn.T.conj() * Ahat)
        pmusic[phi - 1] = num.real / denom.real

    # print max(pmusic)
    #DOA = np.argmax(pmusic)
    count = 0
    for i in pmusic:
        if i == max(pmusic):
            break
        count = count + 1
    DOA = count + 1

if __name__ == "__main__":

    while True:
        (conn, addr) = s.accept()
        data = conn.recv(BUFFER_SIZE)
        print data
        splitMsg(data)
        music_algo()
        print "DOA is: ", DOA, " degrees"
    conn.close()
