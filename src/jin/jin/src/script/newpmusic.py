from numpy import * 
import math
import numpy as np
from numpy import linalg as LA
from numpy import matrix 
import cmath

gamma = [-45, 45, 135, 225]
v = 1500
f = 30000.0
lamda = v / f
d = 0.015
r = math.sqrt(2 * math.pow(d / 2, 2))
A = zeros((4, 1), dtype=complex)
pmusic = zeros((360, 90))
'''
Specify how many sets of data used
for T in take:
    R = np.matrix([[(ls[T])[0]], [(ls[T])[1]],
                   [(ls[T])[2]], [(ls[T])[3]]])
    cov = R * R.T.conj()
    #print cov
    r_conv += cov
'''
#Just a test case
com1 = (0.9389-3.621j)
com2 = (-2.6067-1.7737j)
com3 = (0.4586+2.1283j)
com4 = (-1.9776+2.6057j)
R = np.matrix([[com1], [com2],
               [com3], [com4]])
cov = R*R.T.conj()
#r_conv = r_conv/int(take)
(eigval, eigvec) = LA.eigh(cov)
Vn = eigvec[:, 0:3]

#Theta is altitude
#Phi is azimuth
for theta in range(90):
    for phi in range(360):
        for i in range(4):
            pd = 2*math.pi/lamda*r*math.sin(np.deg2rad(theta))*math.cos(np.deg2rad(phi - gamma[i]))
            A[i] = cmath.exp((1j)*pd)
    
        Ahat = np.matrix(A)
        num = Ahat.T.conj() * Ahat
        denom = (Ahat.T.conj() * Vn) * (Vn.T.conj() * Ahat)
        pmusic[phi, theta] = num.real / denom.real

for i in pmusic:
    print i
#calculate maximum pmusic value
max_val = 0
azimuth = 0
for i in range(360):
    curr_max = max(pmusic[i])
    if max_val < curr_max:
        max_val = curr_max
    else:
        azimuth += 1
DOA = azimuth
altitude = np.argmax(pmusic[azimuth])
print DOA, altitude
