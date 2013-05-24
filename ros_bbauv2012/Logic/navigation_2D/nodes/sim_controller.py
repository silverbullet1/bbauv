#!/usr/bin/env python
import roslib; roslib.load_manifest('move_base_bbauv2012')
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry

def Sim_Controller():
    """ This node subscribes to a topic of type geometry_msgs/Twist, implements a simple P-controller and publishes to UWSim's /setThrusterInput topic

    """

    thrust = { 'T1': 0, 'T2': 0, 'T3': 0, 'T4': 0, 'T5': 0 }
    vel = {'X': 0, 'Y':0, 'Z': 0, 'R': 0, 'P': 0, 'Yaw': 0}
    vel_feedback = {'X': 0, 'Y':0, 'Z': 0, 'R': 0, 'P': 0, 'Yaw': 0}

    def callback(msg):
        vel['X'] = msg.linear.x
        vel['Y'] = msg.linear.y
        vel['Z'] = msg.linear.z
        vel['Yaw'] = msg.angular.z

        
        rospy.loginfo(rospy.get_name() + ": I just converted /cmd_vel to /thruster_input")

    def initVel(msg):
        vel_feedback['X'] = msg.twist.twist.linear.x
        vel_feedback['Y'] = msg.twist.twist.linear.y
        vel_feedback['Z'] = msg.twist.twist.linear.z
        vel_feedback['Yaw'] = msg.twist.twist.angular.z


	#initialize node so roscore know who I amx
    rospy.init_node('Sim_Controller', anonymous=False)
	#declare subscribing from what
    sub = rospy.Subscriber("/cmd_vel", Twist, callback)
    sub_odom = rospy.Subscriber("/odom", Odometry, initVel)
	#declare publishing to what
    pub = rospy.Publisher('/g500/thrusters_input', Float64MultiArray)

	#Publish
    while not rospy.is_shutdown():
        output_topic = Float64MultiArray()

        LinearT1 = LinearT2 = (vel['X'] - vel_feedback['X']) * 10 * -1
        LinearT3 = LinearT4 = (vel['Z'] - vel_feedback['Z']) * 10 * -1
        LinearT5 = (vel['Y'] - vel_feedback['Y']) * 10

        
        AngularT1 = (vel['Yaw'] - vel_feedback['Yaw']) * 3 * -1
        AngularT2 = -1*AngularT1
        
        thrust['T1'] = LinearT1 + AngularT1
        thrust['T2'] = LinearT2 + AngularT2
        thrust['T3'] = LinearT3 + 0.2 #depth controller for simulator not yet done; adding some thrust to keep it at the surface
        thrust['T4'] = LinearT4 + 0.2 #depth controller for simulator not yet done; adding some thrust to keep it at the surface
        thrust['T5'] = LinearT5
            
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
