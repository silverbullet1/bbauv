#!/usr/bin/env python

# Basic ROS imports
import roslib; roslib.load_manifest('navigation_2D')
import rospy
from math import pi
import tf
from tf.transformations import euler_from_quaternion

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from bbauv_msgs.msg import controller
from sensor_msgs.msg import Imu

def main():

    position = { 'x': 0, 'y': 0}
    orientation = {'yaw': 0}
    linear_vel = { 'x': 0, 'y': 0}
    angular_vel = {'z': 0 }
    
    def callback_cmd_vel(msg):
        linear_vel['x'] = msg.linear.x
        linear_vel['y'] = msg.linear.y
        angular_vel['z'] = msg.angular.z*(180/pi)

    def callback_WHDVL(msg):
        position['x'] =  msg.pose.pose.position.x
        position['y'] = msg.pose.pose.position.y

    def callback_AHRS8(msg):

        #angles returned from euler_from_quaternion are in radians
        (roll, pitch, yaw) = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        orientation['yaw'] = yaw*(180/pi)

    def integral(x_dot, x, t):
        """ Computes the integral o x dt """
        return (x_dot * t) + x

    def normalize_angle(angle):
        # Inspiration: http://www.ros.org/doc/api/angles/html/angles_8h_source.html; Normalizes the angle to be 0 to 360 It takes and returns degrees.
        normalized = (angle%360+360)%360
        return normalized        

    #initialize node so roscore know who I am
    rospy.init_node('cmd_position', anonymous=False)

	#declare subscribing from what
    cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, callback_cmd_vel) 
    WH_DVL = rospy.Subscriber("/WH_DVL_data", Odometry, callback_WHDVL)
    AHRS8 = rospy.Subscriber("/AHRS8_data", Imu, callback_AHRS8)

	#declare publishing to what
    cmd_position = rospy.Publisher("/cmd_position", controller)
    
    old_time = rospy.Time.now()

    while not rospy.is_shutdown():

        #code to check for period of each loop in secs
        t1 = rospy.Time.now()
        elapse = (t1-old_time).to_sec()
        print elapse
        old_time = t1

        output = controller()

        new_position_x = integral(linear_vel['x'], position['x'], 1)
        new_position_y = integral(linear_vel['y'], position['y'], 1)

        print "current orientation = %f" % orientation['yaw']
        print "angle to turn = %f" % angular_vel['z']

        new_yaw = normalize_angle(integral(angular_vel['z'], orientation['yaw'], 1))

        print "new yaw = %f" % new_yaw


        output.forward_setpoint = new_position_x
        output.sidemove_setpoint = new_position_y
        output.heading_setpoint = new_yaw
        cmd_position.publish(output)

        t2 = rospy.Time.now()
        #print (t2-t1).to_sec()
        p = 0.2 - (t2-t1).to_sec()
        if p < 0.0 : p = 0.0
        rospy.sleep(p)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
