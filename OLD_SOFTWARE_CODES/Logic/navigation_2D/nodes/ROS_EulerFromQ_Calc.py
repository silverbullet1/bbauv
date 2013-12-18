#!/usr/bin/env python2

import roslib; roslib.load_manifest('navigation_2D')
import rospy
from tf.transformations import quaternion_from_euler, quaternion_about_axis
from math import pi

if __name__ == '__main__':
    rospy.init_node('eFrq', anonymous=True)
    yaw = 90 * (pi/180)
    x,y,z,w = quaternion_from_euler(0,0,yaw)
    rospy.loginfo("Yaw %2.5f is z = %2.5f & w = %2.5f" % (yaw,z,w))
    
