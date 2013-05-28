#!/usr/bin/env python

import roslib; roslib.load_manifest('move_base_bbauv2012')
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry


def PoseToOdom():
    """ This node subscribes to a topic of type geometry_msgs/Pose and converts it to nav_msgs/Odometry

    """

    pose = { 'x': 0, 'y': 0, 'z': 0 }

    def callback(msg):
        pose['x'] = msg.position.x
        pose['y'] = msg.position.y
        #pose['z'] = msg.position.z
        rospy.loginfo(rospy.get_name() + ": I heard Pose data from UWSim")

    #initialize node so roscore know who I am
    rospy.init_node('PoseToOdom', anonymous=False)
	#declare subscribing from what
    sub = rospy.Subscriber("/g500/pose", Pose, callback)
	#declare publishing to what
    pub = rospy.Publisher("/odom", Odometry)

	#Perform conversion
    while not rospy.is_shutdown():
        converted_output = Odometry()
        converted_output.pose.pose.position.x = pose['x']
        converted_output.pose.pose.position.y = pose['y']
        #converted_output.pose.pose.position.z = pose['z']

        #print converted_output	    
        #Publish the data
        pub.publish(converted_output)
        rospy.sleep(0.05)
        #rospy.spin()


	
if __name__ == '__main__':
    try:
        PoseToOdom()
    except rospy.ROSInterruptException:
        pass
