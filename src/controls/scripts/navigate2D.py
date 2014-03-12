#!/usr/bin/env python
import rospy, roslib
from nav_msgs.msg import Odometry
from bbauv_msgs.msg import ControllerGoal, ControllerAction, depth, compass_data
from bbauv_msgs.srv import navigate2d, navigate2dResponse
import numpy as np
import actionlib

#x 2.2 5.7, heading 70

roslib.load_manifest('controls')

class Navigate2D(object):
    def __init__(self):
        self.currPos = {'x' : None, 'y' : None}
        self.currHeading = None
        self.depth = None

        self.DVL = rospy.Subscriber("/earth_odom", Odometry, self.DVLCallback)
        self.AHRS8 = rospy.Subscriber("/euler", compass_data, lambda d:
                                      setattr(self, 'currHeading', d.yaw))
        self.depthSub = rospy.Subscriber("/depth", depth, lambda d:
                                         setattr(self, 'depth', d.depth))
        self.locomotionServer = actionlib.SimpleActionClient("LocomotionServer",
                                                             ControllerAction)
        self.Server = rospy.Service("/navigate2D", navigate2d, self.navCallback)

    def DVLCallback(self, data):
        self.currPos['x'] = data.pose.pose.position.y
        self.currPos['y'] = data.pose.pose.position.x

    def navCallback(self, req):
        rospy.loginfo("Current compass: %f" % (self.currHeading))
        rospy.loginfo("Current position: x:%f, y:%f" % (self.currPos['x'],
                                                        self.currPos['y']))
        rospy.loginfo("Current depth is: %f" % (self.depth))
        (heading, magnitude) = self.getPoints(req.x, req.y)
        try:
            res = self.navigate(heading, magnitude)
        except rospy.ROSException, e:
            rospy.logerr("Navigate2D error: %s" % str(e))
            res = False
        finally:
            return navigate2dResponse(res)


    def getPoints(self, y, x):
        initial = [self.currPos['x'], self.currPos['y']]
        target = [x, y]
        magnitude = np.sqrt(sum(map(lambda k: k * k, list(i - j for i, j in zip(target,
                                                                    initial)))))
        n_vec = list((i - j) for i, j in zip([initial[0], initial[1] + 1], initial))
        #n_vec = [0, 1]
        dv = list((i - j) for i, j in zip(target, initial))
        mag_dv = np.sqrt(sum(map(lambda x: x * x, dv)))
        dotp = sum(list((i * j) for i, j in zip(n_vec, dv)))
        heading = np.rad2deg(np.arccos(dotp/mag_dv))
        if(target[0] < initial[0]):
            heading = 360.0 - heading
        rospy.loginfo("turn %f degs and move %fm" % (heading, magnitude))
        return (heading, magnitude)

    def navigate(self, heading, magnitude):
        goal = ControllerGoal(forward_setpoint=0.0, sidemove_setpoint=0.0,
                              heading_setpoint=heading,
                              depth_setpoint=self.depth)
        rospy.loginfo("Aligning waiting and hoping the locomotionServer is\
                      ready")
        self.locomotionServer.send_goal_and_wait(goal)
        rospy.loginfo("Aligned, going forward")
        goal = ControllerGoal(forward_setpoint=magnitude, sidemove_setpoint=0.0,
                              heading_setpoint=heading,
                              depth_setpoint=self.depth)
        self.locomotionServer.send_goal_and_wait(goal)
        return True


if __name__ == "__main__":
    rospy.init_node("navigate2D", anonymous=False)
    rospy.loginfo("Navigate2D init.")
    nav = Navigate2D()
    rospy.loginfo("Navigate2D main proc")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down navigation 2D node.")
