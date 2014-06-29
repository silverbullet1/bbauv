#!/usr/bin/env python

import rospy, roslib
from bbauv_msgs.msg import dvl_vel
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3

roslib.load_manifest('explorer_dvl')

"""
    temporary integrator before kfilter is up
"""

class Integrator(object):
    def __init__(self):
        rospy.Subscriber('/explorer_vel', dvl_vel, self._velcb)
        self.odom_pub = rospy.Publisher('/WH_DVL_data', Odometry)
        self.oldvx = 0
        self.oldvy = 0
        self.oldvz = 0
        self.currtime = 0
        self.oldtime = 0
        self.x = 0
        self.y = 0;
        self.z = 0
        self.start = 0

    def _velcb(self, msg):
        if self.oldtime == 0:
            self.oldtime = msg.header.stamp
            return
        delta = msg.header.stamp.to_sec() - self.oldtime.to_sec()
        self.oldtime = msg.header.stamp
        self.x += ((self.oldvx + msg.velocity.x) * delta) * 0.5
        self.y += ((self.oldvy + msg.velocity.y) * delta) * 0.5
        self.z += ((self.oldvz + msg.velocity.z) * delta) * 0.5
        self.oldvx = msg.velocity.x
        self.oldvy = msg.velocity.y
        self.oldvz = msg.velocity.z
        position = [self.x, self.y, self.z]
        o = Odometry()
        o.pose.pose.position = Point(*position)
        o.twist.twist.linear = msg.velocity
        o.header.frame_id = 'dvl_position_measured'
        o.header.stamp = msg.header.stamp
        self.odom_pub.publish(o)
        self.start += delta
        print "%f,%f,%f,%f,%f" % (self.start,
                                  msg.velocity.x, self.x, msg.velocity.y,
                                  self.y)



rospy.init_node('velocity_to_distance')
i = Integrator()
rospy.spin()
