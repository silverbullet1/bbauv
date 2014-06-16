#!/usr/bin/env python

import rospy, roslib
from bbauv_msgs.msg import explorer_dvl_data, dvl_vel
from geometry_msgs.msg import Vector3
import numpy as np
from numpy.linalg import inv
from numpy.core import transpose
import math

roslib.load_manifest('explorer_dvl')

rospy.init_node('beamtoxyz')

class BeamToXYZ(object):
    def __init__(self, ea):
        rospy.Subscriber('/explorer_raw_data', explorer_dvl_data, self._rawcb)
        #rospy.Subscriber('/tilt', tilt, lambda d: setattr(self, 'tilt', d))
        self.ea = np.radians(ea)
        self.vel_out = rospy.Publisher('/explorer_vel', dvl_vel)

    def _rawcb(self, msg):
        #beam tilt is output by ps0 command, 600khz 30degs
        s = np.sin(np.radians(30))
        c = np.cos(np.radians(30))
        A = np.zeros((len(msg.beam_velocity), 3))
        A[0] = [-s, 0, -c]
        A[1] = [s,  0, -c]
        A[2] = [0, +s, -c]
        A[3] = [0, -s, -c]
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
