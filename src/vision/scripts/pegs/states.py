#/usr/bin/env/python

'''
Pegs states
'''

import roslib; roslib.load_manifest('vision')
import rospy 

import smach, smach_ros

from comms import Comms

from bbauv_msgs.msg import *
from bbauv_msgsmsgs.srv import *

