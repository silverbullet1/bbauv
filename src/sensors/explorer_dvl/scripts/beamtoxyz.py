#!/usr/bin/env python

import rospy, roslib
from bbauv_msgs.msg import explorer_dvl_data, dvl_vel, compass_data
from geometry_msgs.msg import Vector3
import numpy as np
from numpy.linalg import inv
from numpy.core import transpose
import math
from numpy import radians

roslib.load_manifest('explorer_dvl')

rospy.init_node('beamtoxyz')

sq = lambda d: d * d

class BeamToXYZ(object):
    def __init__(self, ea):
        rospy.Subscriber('/explorer_raw_data', explorer_dvl_data, self._rawcb)
        rospy.Subscriber('/euler', compass_data, self._imucb)
        #rospy.Subscriber('/tilt', tilt, lambda d: setattr(self, 'tilt', d))
        self.ea = np.radians(ea)
        self.beam_tilt = np.radians(30)
        self.vel_out = rospy.Publisher('/explorer_vel', dvl_vel)

    def _imucb(self, msg):
        self.roll = radians(msg.roll)
        self.pitch = radians(msg.pitch)
        t = np.cos(self.beam_tilt) * np.sqrt(
                1 - sq(np.sin(self.roll)) - sq(np.sin(self.pitch))
            )
        beam3 = np.arccos(-np.sin(self.roll) * np.sin(self.beam_tilt) + t)
        beam4 = np.arccos(+np.sin(self.roll) * np.sin(self.beam_tilt) + t)
        beam2 = np.arccos(+np.sin(self.pitch) * np.sin(self.beam_tilt) + t)
        beam1 = np.arccos(-np.sin(self.pitch) * np.sin(self.beam_tilt) + t)
        self.beam_angles = (beam1, beam2, beam3, beam4)

    def _rawcb(self, msg):
        #beam tilt is output by ps0 command, 600khz 30degs
        ba_s = map(np.sin, self.beam_angles)
        ba_c = map(np.cos, self.beam_angles)
        A = np.zeros((len(msg.beam_velocity), 3))
        A[0] = [-ba_s[0], 0, -ba_c[0]]
        A[1] = [ba_s[1],  0, -ba_c[1]]
        A[2] = [0, +ba_s[2], -ba_c[2]]
        A[3] = [0, -ba_s[3], -ba_c[3]]
        b = [x for x in msg.beam_velocity if x is not math.isnan(x)]
        A = np.dot(inv(np.dot(transpose(A), A)), transpose(A))
        x = np.dot(A, b)
        if(abs(x[0]) > 5 or abs(x[1]) > 5 or abs(x[2]) > 5):
            return
        x = [x[1] * np.sin(self.ea) + x[0] * np.sin(self.ea),
            -x[1] * np.cos(self.ea) + x[0] * np.cos(self.ea), x[2]]
        self.vel_out.publish(dvl_vel(
                    header=msg.header,
                    velocity=Vector3(*x)
                ))

b = BeamToXYZ(45.0)
rospy.spin()
