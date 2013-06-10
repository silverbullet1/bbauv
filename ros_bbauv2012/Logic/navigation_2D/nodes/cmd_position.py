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
from bbauv_msgs.msg import imu_data
from sensor_msgs.msg import Imu

def main():

    position = { 'x': 0, 'y': 0}
    orientation = {'yaw': 0}
    linear_vel = { 'x': 0, 'y': 0}
    angular_vel = {'z': 0 }

    def callback_cmd_vel(msg):
        linear_vel['x'] = msg.linear.x #* -1
        linear_vel['y'] = msg.linear.y #* -1
        angular_vel['z'] = msg.angular.z

    def callback_WHDVL(msg):
        position['x'] =  msg.pose.pose.position.x
        position['y'] = msg.pose.pose.position.y 

    def callback_AHRS8(msg):
        orientation['yaw'] = msg.orientation.z *(180/pi) 

    def integral(x_dot, x, t):
        """ Computes the integral o x dt """
        return (x_dot * t) + x

    def normalize_angle(angle):
        # Inspiration: http://www.ros.org/doc/api/angles/html/angles_8h_source.html; Normalizes the angle to be 0 to 360 It takes and returns degrees.
        normalized = (angle%360+360)%360
        return normalized

    def forward_velocity(mb_fwd_vel, curr_Pos):
        lead_distance = 0
        if abs(mb_fwd_vel) == 0:
            lead_distance = 0
        if abs(mb_fwd_vel) <=0.1: 
            lead_distance = 0.5
        if abs(mb_fwd_vel) <=0.2:
            lead_distance = 1
        if abs(mb_fwd_vel) <=0.3:
            lead_distance = 2
        if mb_fwd_vel > 0:
            forward_setpoint = lead_distance + curr_Pos
        if mb_fwd_vel < 0:
            forward_setpoint = -1 * lead_distance + curr_Pos
        return forward_setpoint 

    def sidemove_velocity(mb_side_vel, curr_Pos):
        sidemove_setpoint = 0
        lead_distance = 0
        if abs(mb_side_vel) == 0:
            lead_distance = 0
        if abs(mb_side_vel) <=0.1: 
            lead_distance = 0.5
        if abs(mb_side_vel) <=0.2:
            lead_distance = 1
        if mb_side_vel > 0:
            sidemove_setpoint = lead_distance + curr_Pos
        if mb_side_vel < 0:
            sidemove_setpoint = -1 * lead_distance + curr_Pos
        return sidemove_setpoint

    def yaw_velocity(mb_yaw_vel, curr_Yaw):
        lead_angle = 0
        if abs(mb_yaw_vel) == 0:
            lead_angle = 0
        if abs(mb_yaw_vel) <=0.1: #5
            lead_angle = 2.5
        if abs(mb_yaw_vel) <=0.15: #5
            lead_angle = 5
        if abs(mb_yaw_vel) <=0.2: #10
            lead_angle = 10
        if abs(mb_yaw_vel) <=0.4: #20
            lead_angle = 20
        if abs(mb_yaw_vel) <=0.6: #30
            lead_angle = 30
        if abs(mb_yaw_vel) <=0.7: #40
            lead_angle = 40
        if abs(mb_yaw_vel) >=0.7: #40
            lead_angle = 40
        if mb_yaw_vel > 0:
            yaw_setpoint = normalize_angle(-1 * lead_angle + curr_Yaw)
        if mb_yaw_vel < 0:
            yaw_setpoint = normalize_angle(lead_angle + curr_Yaw)
        return yaw_setpoint

    #initialize node so roscore know who I am
    rospy.init_node('cmd_position', anonymous=False)

	#declare subscribing from what
    cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, callback_cmd_vel) 
    WH_DVL = rospy.Subscriber("/WH_DVL_data", Odometry, callback_WHDVL)
    AHRS8 = rospy.Subscriber("/AHRS8_data_e", imu_data, callback_AHRS8)

	#declare publishing to what
    cmd_position = rospy.Publisher("/cmd_position", controller)
    
    old_time = rospy.Time.now()
    current_y = position['y']    
    while not rospy.is_shutdown():

#        #code to check for period of each loop in secs
#        t1 = rospy.Time.now()
#        elapse = (t1-old_time).to_sec()
#        #print elapse
#        old_time = t1

        output = controller()
        #lead_distance = 1
        lead_angle = -15 #degrees
    	lead_angle_onspot = -10

        if linear_vel['x'] == None and angular_vel['z']== None:
            rospy.loginfo("No mBase Cmds")

        if linear_vel['x']==0 and angular_vel['z']==0:
            current_x = position['x']
#            current_y = position['y']
            current_yaw = orientation['yaw']
            while linear_vel['x']==0 and angular_vel['z']==0 and not rospy.is_shutdown():
                #code to check for period of each loop in secs
                t1 = rospy.Time.now()
                elapse = (t1-old_time).to_sec()
                #print elapse
                old_time = t1

                output.forward_setpoint = current_x
                output.sidemove_setpoint = current_y
                output.heading_setpoint = current_yaw
                cmd_position.publish(output)

                t2 = rospy.Time.now()
                #print (t2-t1).to_sec()
                p = 0.05 - (t2-t1).to_sec()
                if p < 0.0 : p = 0.0
                rospy.sleep(p)
            rospy.loginfo("Exiting Stationary")
            current_y = position['y']

        if linear_vel['x']!=0 and angular_vel['z']!=0 :
            #current_y = position['y']
            while linear_vel['x']!=0 and angular_vel['z']!=0 and not rospy.is_shutdown():
                #code to check for period of each loop in secs
                t1 = rospy.Time.now()
                elapse = (t1-old_time).to_sec()
                #print elapse
                old_time = t1
                
                output.forward_setpoint = forward_velocity(linear_vel['x'], position['x'])
                output.sidemove_setpoint = current_y
#                output.heading_setpoint = current_yaw

                output.heading_setpoint = yaw_velocity(angular_vel['z'], orientation['yaw'])

#                if angular_vel['z']>0:
#                    output.heading_setpoint = normalize_angle(lead_angle_onspot + orientation['yaw']) #normalize_angle(integral(converted_angular_vel['z'], orientation['yaw'], 0.05))
#                if angular_vel['z']<0:
#                    output.heading_setpoint = normalize_angle(-1 * lead_angle_onspot + orientation['yaw']) #normalize_angle(integral(converted_angular_vel['z'], orientation['yaw'], 0.05))
#                rospy.loginfo("Error on AngZ= %2.5f", output.heading_setpoint-orientation['yaw'])

                cmd_position.publish(output)

                t2 = rospy.Time.now()
                #print (t2-t1).to_sec()
                p = 0.05 - (t2-t1).to_sec()
                if p < 0.0 : p = 0.0
                rospy.sleep(p)
            output.forward_setpoint = position['x'] + 0.45
            cmd_position.publish(output)


        if linear_vel['x']==0 and angular_vel['z']!=0:
            #current_x = position['x']
            #current_y = position['y']
            while linear_vel['x']==0 and angular_vel['z']!=0 and not rospy.is_shutdown():
                #code to check for period of each loop in secs
                t1 = rospy.Time.now()
                elapse = (t1-old_time).to_sec()
                #print elapse
                old_time = t1

                output.forward_setpoint = current_x
                output.sidemove_setpoint = current_y
                output.heading_setpoint = yaw_velocity(angular_vel['z'], orientation['yaw'])
#                if angular_vel['z']>0:
#                    output.heading_setpoint = normalize_angle(lead_angle + orientation['yaw']) #normalize_angle(integral(converted_angular_vel['z'], orientation['yaw'], 0.05))
#                if angular_vel['z']<0:
#                    output.heading_setpoint = normalize_angle(-1 * lead_angle + orientation['yaw']) #normalize_angle(integral(converted_angular_vel['z'], orientation['yaw'], 0.05))
#                rospy.loginfo("Error on AngZ= %2.5f", output.heading_setpoint-orientation['yaw'])
                cmd_position.publish(output)

                t2 = rospy.Time.now()
                #print (t2-t1).to_sec()
                p = 0.05 - (t2-t1).to_sec()
                if p < 0.0 : p = 0.0
                rospy.sleep(p)

        if linear_vel['x']!=0 and angular_vel['z']==0:
            #current_y = position['y']
            current_yaw = orientation['yaw']
            while linear_vel['x']!=0 and angular_vel['z']==0 and not rospy.is_shutdown():
                #code to check for period of each loop in secs
                t1 = rospy.Time.now()
                elapse = (t1-old_time).to_sec()
                #print elapse
                old_time = t1

                output.forward_setpoint = forward_velocity(linear_vel['x'], position['x'])

#                if linear_vel['x']>0:
#                    output.forward_setpoint = lead_distance + position['x'] # + integral(linear_vel['x'], position['x'], 0.05) + lead_distance
#                if linear_vel['x']<0:
#                    output.forward_setpoint = -1 * lead_distance + position['x'] # + integral(linear_vel['x'], position['x'], 0.05) + lead_distance

                rospy.loginfo("Error on X= %2.5f", output.forward_setpoint-position['x'])
                output.sidemove_setpoint = current_y
                output.heading_setpoint = current_yaw
                cmd_position.publish(output)

                t2 = rospy.Time.now()
                #print (t2-t1).to_sec()
                p = 0.05 - (t2-t1).to_sec()
                if p < 0.0 : p = 0.0
                rospy.sleep(p)
            
            output.forward_setpoint = position['x'] + 0.45
            cmd_position.publish(output)   

#        t2 = rospy.Time.now()
#        #print (t2-t1).to_sec()
#        p = 0.05 - (t2-t1).to_sec()
#        if p < 0.0 : p = 0.0
        rospy.sleep(0)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#        if angular_vel['z'] > 0: lead_angle = 10
#        if angular_vel['z'] < 0: lead_angle = -10
#        if angular_vel['z'] == 0: lead_angle = 0
