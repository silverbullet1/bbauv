#!/usr/bin/env python
import rospy, roslib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from sensor_msgs.msg import Imu
import numpy as np
import actionlib

roslib.load_manifest('controls')

"""
TODO
====

    Test node
    Make sure angle calculations are accurate for every quadrant
    Add timeouts
    Change temporary lambdas to full fledged callbacks that actually check for
    status
    Test the Service node
    When sending goal, SEND CURRENT DEPTH (IMPORTANT IMPORTANT etc)
                       ============================================

    Check for Imu (vs imu_data) and Twist imports
    Ask TC how to reset the DVL
"""

class Navigate2D(object):
    def __init__(self):
        self.currPos = {'x' : 0,
                        'y' : 0}
        self.currHeading = {'yaw' : 0}
        self.WH_DVL = rospy.Subscriber('/WH_DVL_data', Odometry,
                                       self.DVLCallback)
        self.AHRS8 = rospy.Subscriber('/AHRS8_data_e', imu_data,
                                      self.CompassCallback)
        try:
            self.ControllerSettings = rospy.ServiceProxy("/set_controller_srv",
                                                        set_controller)
            rospy.loginfo("Waiting for Controller Service")
            self.ControllerSettings.wait_for_service()
            rospy.loginfo("Got Controller Service, asking for forward, heading, \
                        depth control and pitch")
            self.ControllerSettings(forward=True, sidemove=False, heading=True,
                                    depth=False, roll=False, topside=False,
                                    navigation=False)
        except rospy.ServiceException:
            rospy.logerr("Error subscribing to controller")

        try:
            self.actionClient = actionlib.SimpleActionClient('LocomotionServer',
                                                             ControllerAction)
        except rospy.ServiceException:
            rospy.logerr("Cannot proc actionClient")

        rospy.loginfo("Preparing to open server to recieve x, y")
        self.Server = rospy.Service("/navigate2D", navigate2d,
                                    self.handleServer)


    def DVLCallback(self, data):
        self.currPos['x'] = data.pose.pose.position.x
        self.currPos['y'] = data.pose.pose.position.y

    def CompassCallback(self, data):
        self.currHeading['yaw'] = np.rad2deg(data.orientation.z)
    
    @staticmethod
    def normalize_angle(angle):
        return (angle % 360 + 360) % 360

    @staticmethod
    def magnitude(x, y, z=0):
        return np.sqrt((x * x) + (y * y) + (z * z))

    def navigateToPoint(self, x, y):
        """
        distance to travel to is the magnitude of the direction vector
        angle is absolute to true north.
        always use atan2
        atan2 syntax arctan2(y points, x points) -> radians
        """
        distance = self.magnitude(x - self.currPos['x'], y - self.currPos['y'])
        angle = 180.0 + np.rad2deg((np.arctan2(y - self.currPos['y'], x -
                                               self.currPos['x'])))

        rospy.loginfo("We are about to turn to %f degrees and move %f meteres\
                      forward" % (angle, distance))

        goal = ControllerGoal(heading_setpoint=angle)
        
        rospy.loginfo("Waiting for actionserver before turn goal")
        self.actionClient.wait_for_server()
        rospy.loginfo("sending goal to turn")
        self.actionClient.send_goal(goal, lambda s, r:
                                    rospy.loginfo(str(s)))
        self.actionClient.wait_for_result()

        goal = ControllerGoal(forward_setpoint=distance)
        self.actionClient.wait_for_server()
        self.actionClient.send_goal(goal, lambda s, r:
                                    rospy.loginfo(str(s)))
        self.actionClient.wait_for_result()

    def returnToBase(self):
        return self.navigateToPoint(0, 0)

    def handleServer(self, r):
        self.navigateToPoint(r.x, r.y)
        return navigate2dResponse(done=True)

if __name__ == "__main__":
    rospy.init_node('navigate2d', anonymous=False)
    nav = Navigate2D()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down navigate node")

