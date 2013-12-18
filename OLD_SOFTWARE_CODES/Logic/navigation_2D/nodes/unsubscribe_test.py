#!/usr/bin/env python

import roslib; roslib.load_manifest('navigation_2D')
import rospy

from nav_msgs.msg import Odometry

class Unsubscribe_Test():
    
    def __init__(self):
        rospy.init_node("unsub_test", anonymous=False)
        self.sub = None
        self.DVL_x = 0
        self.DVL_y = 0
        
    def callback_WHDVL(self,msg):
        self.DVL_x =  msg.pose.pose.position.x
        self.DVL_y = msg.pose.pose.position.y
        rospy.loginfo("Subsribe ok: %2.5f %2.5f" % (self.DVL_x, self.DVL_y)) 
    
    def subscribe(self):
        self.sub = rospy.Subscriber("/WH_DVL_data", Odometry, self.callback_WHDVL)
        rospy.loginfo("Subscribing")
        
    def unsubscribe(self):
        self.sub.unregister()
        while True and not rospy.is_shutdown():
            rospy.loginfo("Unsubscribe ok: %2.5f %2.5f" % (self.DVL_x, self.DVL_y)) 
        
        
if __name__ == '__main__':
    
    test = Unsubscribe_Test()
    test.subscribe()
    rospy.sleep(10)
    test.unsubscribe()
    
         
    
            
    
    
    
