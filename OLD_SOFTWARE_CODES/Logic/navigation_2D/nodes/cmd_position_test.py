#!/usr/bin/env python

# Basic ROS imports
import roslib; roslib.load_manifest('navigation_2D')
import rospy
from geometry_msgs.msg import Twist

def main():

    rospy.init_node('cmd_position_test', log_level=rospy.DEBUG, anonymous=False)
    #declare publishing to what
    cmd_vel_test = rospy.Publisher("/cmd_vel", Twist)
    x = 0
    while not rospy.is_shutdown():
        output = Twist()
        
        output.linear.x = 0
        output.linear.y = 0
        output.linear.z = 0
        output.angular.z = 0
        cmd_vel_test.publish(output)
        if x<=0.3:
            x += 0.1
        else:
            x -= 0.1
     	rospy.sleep(0.2)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


