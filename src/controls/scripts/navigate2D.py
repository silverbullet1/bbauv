#!/usr/bin/env python
import rospy, roslib
from nav_msgs.msg import Odometry
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

    Check for Imu (vs imu_data) and Twist imports
    Ask TC how to reset the DVL
"""

class Navigate2D(object):
    def __init__(self):
        self.currPos = {'x' : 0,
                        'y' : 0}
        self.currHeading = {'yaw' : 0}
        self.depth = 0
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

        rospy.Subscriber("/depth", depth, lambda d: setattr(self, 'depth', d))

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

    def handleServer(self, r):
        res = self.navigateToPoint(r.x, r.y)
        return navigate2dResponse(done=res)

    def navigateToPoint(self, x, y):
        initial = list(self.currPos['x'], self.currPos['y'])
        target = list(x, y)
        magnitude = np.sqrt(sum(map(lambda k: k * k, list(i - j for i, j in zip(target,
                                                                    initial)))))
        n_vec = list((i - j) for i, j in zip([initial[0], initial[1] + 1], initial))
        dv = list((i - j) for i, j in zip(target, initial))
        mag_dv = np.sqrt(sum(map(lambda x: x * x, dv)))
        dotp = sum(list((i * j) for i, j in zip(n_vec, dv)))
        heading = np.rad2deg(np.arccos(dotp/mag_dv))
        if(target[0] < initial[0]):
            heading = 360.0 - heading
        rospy.loginfo("turn %f degs and move %fm" % (heading, magnitude))

        rospy.loginfo("Waiting for actionclient before sending goal to turn")
        self.actionClient.wait_for_server()
        rospy.loginfo("sending turn setpoint")
        goal = ControllerGoal(heading_setpoint=heading,
                              depth_setpoint=self.depth)
        self.actionClient.send_goal(goal, lambda st, res:
                                    rospy.loginfo("done diving: %s %s" %
                                                  (str(res),
                                                   actionlib.GoalStatus.SUCCEEDED
                                                   == st)))
        self.actionClient.wait_for_result()

        goal = ControllerGoal(forward_setpoint=magnitude,
                              depth_setpoint=self.depth)
        self.actionClient.wait_for_server()
        self.actionClient.send_goal(goal, lambda st, res:
                                    rospy.loginfo("dont moving forward?: %s %s" %
                                                  (str(res),
                                                   actionlib.GoalStatus.SUCCEEDED
                                                  == st)))
        self.actionClient.wait_for_result()
        return True



if __name__ == "__main__":
    rospy.init_node('navigate2d', anonymous=False)
    nav = Navigate2D()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down navigate node")

