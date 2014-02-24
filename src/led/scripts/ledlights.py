#!/usr/bin/env python

#Rotate through the LED light strips

from bbauv_msgs.msg import *
from std_msgs.msg import Int8

import roslib; roslib.load_manifest('led')
import rospy

import signal

class LED:
    def __init__(self): 
        self.light = 0  
        self.led_pub = rospy.Publisher("/led_strips", Int8)
        
        signal.signal(signal.SIGINT, self.userQuit)
        
        self.dance()
        
    def userQuit(self, signal, frame):
        rospy.signal_shutdown("Bye!")
    
    def dance(self):
        while not rospy.is_shutdown():
            self.led_pub.publish(self.light)
            self.light = self.light + 1
            if self.light > 9:
                self.light = 0
            rospy.loginfo(self.light)
            rospy.sleep(0.3)

if __name__ == "__main__":
    rospy.init_node("LED")
    ledLights = LED()
    rospy.spin()
        