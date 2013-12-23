#!/usr/bin/env python

# Basic ROS imports
import roslib; roslib.load_manifest('navigation_2D')
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

def main():
	#declare subscribing from what
    cmd_vel_sub = rospy.Publisher("/cmd_vel", Twist) 
    WH_DVL = rospy.Publisher("/WH_DVL_data", Odometry)
    AHRS8 = rospy.Publisher("/AHRS8_data", Imu)
    
    counter = 0 
    vel = Twist()
    dvl = Odometry()
    ahrs = Imu()

    vel.linear.x = 0
    vel.linear.y = 0
    vel.angular.z = 0
    dvl.pose.pose.position.x = 10
    dlv.pose.pose.position.y = 5
    
    while not rospy.is_shutdown():
        
        if counter < 10:
            vel.linear.x += 1
            vel.linear.y += 1
            vel.angular.z += 0.5

            counter += 1
        if counter > 10:
            vel.linear.x -= 1
            vel.linear.y -= 1
            vel.angular.z -= 0.5
            counter -= 1

        cmd_vel_sub.publish(vel)
        
