#!/usr/bin/env python

import rospy, roslib
from sensor_msgs.msg import Imu
from bbauv_msgs.msg import tilt
from math import sqrt, acos
from numpy import degrees

roslib.load_manifest('explorer_dvl')

rospy.init_node('tilt_from_gyro')

tilt_pub = rospy.Publisher('tilt', tilt);

def cb(data):
    x = [data.linear_acceleration.x, data.linear_acceleration.y,
         data.linear_acceleration.z]
    res = degrees(acos(x[2] / sqrt(sum(map(lambda d: d * d, x)))))
    tilt_pub.publish(tilt(
            header=data.header,
            tilt=res
        ))


rospy.Subscriber('/imu_data_q', Imu, cb)
rospy.spin()
