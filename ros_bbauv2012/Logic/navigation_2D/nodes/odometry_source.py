#!/usr/bin/env python

# Basic ROS imports
import roslib; roslib.load_manifest('navigation_2D')
import rospy
import PyKDL
import sys

# import msgs
from std_msgs.msg import Float64MultiArray 
from geometry_msgs.msg import Pose 
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

# More imports
from numpy import *
import tf

#I want to:
# combine AHRS8 and DVL sensor data into a single nav::msgs /odom message type
# broadcast /odom to /base_footprint

def main():
    
    header = {'seq': 0, 'secs': 0, 'nsecs': 0, 'frame_id': "odom", 'child_frame_id': "base_footprint"}
    position = { 'x': 0, 'y': 0, 'z': 0 }
    orientation = { 'x':0, 'y':0, 'z':0, 'w':0}
    linear_vel = { 'x': 0, 'y': 0, 'z': 0 }
    angular_vel = { 'x': 0, 'y': 0, 'z': 0 }

    def callback_WHDVL(msg):
        header['seq'] = msg.header.seq
        header['secs'] = msg.header.stamp.secs
        header['nsecs'] = msg.header.stamp.nsecs
        position['x'] =  msg.pose.pose.position.x
        position['y'] = msg.pose.pose.position.y
        position['z'] = 0 #msg.pose.pose.position.z
        linear_vel['x'] = msg.twist.twist.linear.x
        linear_vel['y'] = msg.twist.twist.linear.y
        linear_vel['z'] = 0 #msg.twist.twist.linear.z

    def callback_AHRS8(msg):
        orientation['x'] = msg.orientation.x 
        orientation['y'] = msg.orientation.y
        orientation['z'] = msg.orientation.z
        orientation['w'] = msg.orientation.w
        angular_vel['x'] = msg.angular_velocity.x
        angular_vel['y'] = msg.angular_velocity.y
        angular_vel['z'] = msg.angular_velocity.z

    #initialize node so roscore know who I am
    rospy.init_node('Odometry_Source', anonymous=False)

	#declare subscribing from what
    WH_DVL = rospy.Subscriber("/WH_DVL_data", Odometry, callback_WHDVL)    
    AHRS8 = rospy.Subscriber("/AHRS8_data", Imu, callback_AHRS8)

	#declare publishing to what
    odometry = rospy.Publisher("/odom", Odometry)

    while not rospy.is_shutdown():

        # Broadcast transform
        br = tf.TransformBroadcaster()
        br.sendTransform((position['x'], position['y'], position['z']), (orientation['x'], orientation['y'],orientation['z'], orientation['w']), 
        rospy.Time.now(), "/base_footprint", "/odom")
        
        # Filling in output message type
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
                                                        
        rospy.sleep(0.2)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


