#!/usr/bin/env python
import roslib; roslib.load_manifest('move_base_bbauv2012')
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from bbauv_msgs.msg import thruster

def Sim_Controller():
    """ This node subscribes to a topic of type geometry_msgs/Twist, implements a simple P-controller and publishes to UWSim's /setThrusterInput topic

    """

    thrust = { 'T1': 0, 'T2': 0, 'T3': 0, 'T4': 0, 'T5': 0 }
    speed = {'T1': 0, 'T2':0, 'T3': 0, 'T4': 0, 'T5': 0, 'T6': 0}
    vel = {'X': 0, 'Y':0, 'Z': 0, 'R': 0, 'P': 0, 'Yaw': 0}
    vel_feedback = {'X': 0, 'Y':0, 'Z': 0, 'R': 0, 'P': 0, 'Yaw': 0}

    def callback(msg):
        speed['T1'] = msg.speed1
        speed['T2'] = msg.speed2
        speed['T3'] = msg.speed3
        speed['T4'] = msg.speed4
        speed['T5'] = msg.speed5
        speed['T6'] = msg.speed6

    def callback2(msg):
        vel['X'] = msg.linear.x
        vel['Y'] = msg.linear.y
        vel['Z'] = msg.linear.z
        vel['Yaw'] = msg.angular.z

    def initVel(msg):
        vel_feedback['X'] = msg.twist.twist.linear.x
        vel_feedback['Y'] = msg.twist.twist.linear.y
        vel_feedback['Z'] = msg.twist.twist.linear.z
        vel_feedback['Yaw'] = msg.twist.twist.angular.z
        rospy.loginfo(rospy.get_name() + ": I just converted /cmd_vel to /thruster_input")

	#initialize node so roscore know who I am
    rospy.init_node('Sim_Controller_BBAUV', anonymous=False)
	#declare subscribing from what
    sub = rospy.Subscriber("/thruster_feedback", thruster, callback)
    sub2 = rospy.Subscriber("/cmd_vel", Twist, callback2)
    sub_odom = rospy.Subscriber("/odom", Odometry, initVel)

	#declare publishing to what
    pub = rospy.Publisher('/g500/thrusters_input', Float64MultiArray)

	#Publish
    while not rospy.is_shutdown():
        output_topic = Float64MultiArray()

        LinearT5 = (vel['Y'] - vel_feedback['Y']) * 10

        thrust['T1'] = speed['T1']/2560.0
        thrust['T2'] = speed['T2']/2560.0
        thrust['T3'] = 0.2 #depth controller for simulator not yet done; adding some thrust to keep it at the surface
        thrust['T4'] = 0.2 #depth controller for simulator not yet done; adding some thrust to keep it at the surface
        thrust['T5'] = LinearT5 #speed['T1']/2560.0
            
        output_list = []
        output_list.append(thrust['T1'])
        output_list.append(thrust['T2'])
        output_list.append(thrust['T3'])
        output_list.append(thrust['T4'])
        output_list.append(thrust['T5'])

        output_topic.data = output_list

        #Publish the data
        pub.publish(output_topic)
        rospy.sleep(0.05)
        #rospy.spin()


	
if __name__ == '__main__':
    try:
        Sim_Controller()
    except rospy.ROSInterruptException:
        pass
