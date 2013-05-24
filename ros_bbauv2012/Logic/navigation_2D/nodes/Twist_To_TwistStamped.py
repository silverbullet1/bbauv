#!/usr/bin/env python
import roslib; roslib.load_manifest('move_base_bbauv2012')
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped


def Twist_To_TwistStamped():
    """ This node subscribes to a topic of type geometry_msgs/Twist and converts it to geometry_msgs/TwistStamped

    Originally written to send data from /cmd_vel topic into /g500/twist (as seen in the default launch of UWSim)
    """

    twist = { 'x': 0, 'ang_z': 0 }

    def callback(msg):
        twist['x'] = msg.linear.x
        twist['ang_z'] = msg.angular.z   
        rospy.loginfo(rospy.get_name() + ": I heard Twist data from move_base's cmd_vel")

	#initialize node so roscore know who I am
    rospy.init_node('Twist_To_TwistStamped', anonymous=False)
	#declare subscribing from what
    sub = rospy.Subscriber("/cmd_vel", Twist, callback)
	#declare publishing to what
    pub = rospy.Publisher('/g500/twist', TwistStamped)

	#Perform conversion
    while not rospy.is_shutdown():
        converted_output = TwistStamped()
        converted_output.twist.linear.x = twist['x']
        converted_output.twist.angular.z = twist['ang_z']

        #print converted_output	    
        #Publish the data
        pub.publish(converted_output)
        rospy.sleep(0.05)
        #rospy.spin()


	
if __name__ == '__main__':
    try:
        Twist_To_TwistStamped()
    except rospy.ROSInterruptException:
        pass
