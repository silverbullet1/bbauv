#!/usr/bin/env python

# Basic ROS imports
import roslib; roslib.load_manifest('navigation_2D')
import rospy
import PyKDL
import sys
from math import pi

# import msgs
from std_msgs.msg import Float32
from std_msgs.msg import Float64MultiArray 
from geometry_msgs.msg import Pose 
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from bbauv_msgs.msg import imu_data

# More imports
from numpy import *
import tf
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

# Publishes transform between /odom and /base_footprint

def main():
    
    header = {'seq': 0, 'secs': 0, 'nsecs': 0, 'frame_id': "odom", 'child_frame_id': "base_footprint"}
    position = { 'x': 0, 'y': 0, 'z': 0 }
    orientation = { 'x':0, 'y':0, 'z':0, 'w':0}
    linear_vel = { 'x': 0, 'y': 0, 'z': 0 }
    angular_vel = { 'x': 0, 'y': 0, 'z': 0 }

    def callback_earthodom(msg):
        header['seq'] = msg.header.seq
        header['secs'] = msg.header.stamp.secs
        header['nsecs'] = msg.header.stamp.nsecs
        #converting NED axis to ROS axes convention
        position['x'] =  msg.pose.pose.position.x # * -1
        position['y'] = msg.pose.pose.position.y * -1
#         position['z'] = msg.pose.pose.position.z #* -1
#        linear_vel['x'] = msg.twist.twist.linear.x
#        linear_vel['y'] = msg.twist.twist.linear.y # * -1
#        linear_vel['z'] = msg.twist.twist.linear.z #* -1

    def callback_AHRS8(msg):
        orientation['x'] = msg.orientation.x #* -1
        orientation['y'] = msg.orientation.y #* -1
        orientation['z'] = msg.orientation.z * -1
        orientation['w'] = msg.orientation.w

#        angular_vel['x'] = msg.angular_velocity.x 
#        angular_vel['y'] = msg.angular_velocity.y #* -1
#        angular_vel['z'] = msg.angular_velocity.z #* -1

    def callback_Altitude(msg):
        position['z'] = msg.data #* -1        

    def normalize_angle(angle):
        # Inspiration: http://www.ros.org/doc/api/angles/html/angles_8h_source.html; Normalizes the angle to be 0 to 360 It takes and returns degrees.
        normalized = (angle%360+360)%360
        return normalized              

    #initialize node so roscore know who I am
    rospy.init_node('Odometry_Source', anonymous=False)

	#declare subscribing from what
    WH_DVL = rospy.Subscriber("/earth_odom", Odometry, callback_earthodom)    
    AHRS8 = rospy.Subscriber("/AHRS8_data_q", Imu, callback_AHRS8)
    Altitude = rospy.Subscriber("/altitude", Float32, callback_Altitude)
    
	#declare publishing to what
    odometry = rospy.Publisher("/odom", Odometry)

    while not rospy.is_shutdown():
        # Broadcast transform
        br = tf.TransformBroadcaster()
        br.sendTransform((position['x'], position['y'], position['z']), (orientation['y'], orientation['x'],orientation['z'], orientation['w']), 
        rospy.Time.now(), "/base_footprint", "/odom")
        rospy.logdebug("Sending Transform")

        output = Odometry()
        output.header.seq = header['seq']
        output.header.stamp.secs = header['secs']
        output.header.stamp.nsecs = header['nsecs']
        output.header.frame_id = header['frame_id']
        output.child_frame_id = header['child_frame_id']
        output.pose.pose.position.x = position['x']
        output.pose.pose.position.y = position['y']
        output.pose.pose.position.z = position['z']
        output.pose.pose.orientation.x = orientation['x']
        output.pose.pose.orientation.y = orientation['y']
        output.pose.pose.orientation.z = orientation['z']                
        output.pose.pose.orientation.w = orientation['w']
        output.twist.twist.linear.x = linear_vel['x']
        output.twist.twist.linear.y = linear_vel['y']
        output.twist.twist.linear.z = linear_vel['z']
        output.twist.twist.angular.x = angular_vel['x']
        output.twist.twist.angular.y = angular_vel['y']
        output.twist.twist.angular.z = angular_vel['z']
        odometry.publish(output)
                                                        
        rospy.sleep(0.05)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

        
        # Filling in output message type

