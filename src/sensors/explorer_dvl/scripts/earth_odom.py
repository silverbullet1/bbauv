#!/usr/bin/env python
import rospy, roslib
from nav_msgs.msg import Odometry
from bbauv_msgs.msg import compass_data
from numpy import radians, sin, cos, pi

from dynamic_reconfigure.server import Server as DynServer
from explorer_dvl.cfg import earth_odomConfig

roslib.load_manifest('explorer_dvl')

class EarthOdom(object):
    def __init__(self):
        rospy.Subscriber('/WH_DVL_data', Odometry, self._ondvl)
        rospy.Subscriber('/euler', compass_data, self._onimu)
        self.epub = rospy.Publisher('/earth_odom', Odometry)
        self.old_data = None
        self.yaw = None
        self.x = 0
        self.y = 0
        self.dynServer = DynServer(earth_odomConfig, self.reconfcb)
        self.time = 0.0
        
    
    def _ondvl(self, msg):
        if self.old_data is None:
            self.old_data = msg
            return

        if self.yaw is None:
            rospy.logerr('no yaw reading, check imu')
            return

        delta = msg.header.stamp.to_sec() - self.old_data.header.stamp.to_sec()
        velx = msg.twist.twist.linear.x
        vely = msg.twist.twist.linear.y

        northv = vely * cos(self.yaw + (pi/2.0)) + velx * sin(self.yaw)
        eastv  = vely * sin(self.yaw + (pi/2.0)) + velx * cos(self.yaw)


        msg.twist.twist.linear.x = northv
        msg.twist.twist.linear.y = eastv

        if msg.twist.twist.linear.x > 0.5:
            print msg.twist.twist.linear.x
        if msg.twist.twist.linear.y > 0.5:
            print msg.twist.twist.linear.y

        self.x += (self.old_data.twist.twist.linear.x +
                   msg.twist.twist.linear.x) * delta * 0.5;
        self.y += (self.old_data.twist.twist.linear.y +
                   msg.twist.twist.linear.y) * delta * 0.5;
        self.z = msg.pose.pose.position.z

        o = Odometry()
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.position.z = msg.pose.pose.position.z
        o.twist.twist.linear.x = velx
        o.twist.twist.linear.y = vely
        o.twist.twist.linear.z = msg.twist.twist.linear.z
        o.header.stamp = msg.header.stamp
        o.header.frame_id = "earth_link"

        print "%f,%f,%f,%f,%f" % (self.time,northv, self.x, eastv, self.y)

        self.epub.publish(o)
        self.old_data = msg
        self.time += delta

    def _onimu(self, msg):
        self.yaw = radians(msg.yaw)

    def reconfcb(self, config, level):
        if(config.reset_earth):
            rospy.loginfo("earth odom reset")
            self.x = 0
            self.y = 0
        config.reset_earth = False
        return config

rospy.init_node('earth_odom', anonymous=False)
e = EarthOdom()
rospy.spin()

