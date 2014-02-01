#!/usr/bin/env python

import rospy
from bbauv_msgs.srv import *
from bbauv_msgs.msg import *

def handle_add_two_ints(req):
    print "Returning [%s + %s]"%(req.start_request, req.abort_request)
    return mission_linefollowerResponse(True, True)

def add_two_ints_server():
    rospy.init_node('LINEFOLLOWER')
    s = rospy.Service('linefollower_service', mission_linefollower, handle_add_two_ints)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
