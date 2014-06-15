#!/usr/bin/env python
import rospy, roslib
from nav_msgs.msg import Odometry
from bbauv_msgs.msg import depth, ControllerAction, ControllerGoal
from bbauv_msgs.msg import compass_data
from bbauv_msgs.srv import set_controller, navigate2d, navigate2dResponse
import actionlib, numpy as np

roslib.load_manifest('controls')

class Navigation(object):
    def __init__(self):
        self.heading = None
        self.cpos = {'x' : None,
                     'y' : None}
        self.depth = None
        self.heading = None
        self.fheading = None

        rospy.loginfo("Initializing subscribers")
        self.earth_odom = rospy.Subscriber("/earth_odom", Odometry,
                                          self.collectOdom)
        self.depth_sub = rospy.Subscriber("/depth", depth, lambda d:
                                         setattr(self, 'depth', d.depth))
        self.heading = rospy.Subscriber("/euler", compass_data, lambda d:
                                       setattr(self, 'heading', d.yaw))

        rospy.loginfo("Getting controller")
        self.initController = rospy.ServiceProxy("/set_controller_srv",
                                                set_controller)
        self.initController.wait_for_service()
        self.controller = actionlib.SimpleActionClient("LocomotionServer",
                                                      ControllerAction)

        rospy.loginfo("Populating sensors")
        while self.depth is None:
            rospy.sleep(1)
        while self.cpos['x'] is None:
            rospy.sleep(1)
        while self.cpos['y'] is None:
            rospy.sleep(1)
        rospy.loginfo("Sensors populated")

        rospy.loginfo("Spawing service server")
        self.navServer = rospy.Service("/navigate2D", navigate2d,
                                      self.navCallback)
        rospy.loginfo("Done")

    def collectOdom(self, d):
        self.cpos['x'] = d.pose.pose.position.y
        self.cpos['y'] = d.pose.pose.position.x

    def navCallback(self, req):
        (heading, distance) = self.getPoints(req.x, req.y)
        self.enable_PID()
        rospy.loginfo("Requested point (%s, %s)", str(req.x), str(req.y))
        rospy.loginfo("Computed heading, distance: (%s, %s)", str(heading),
                     str(distance))

        goal = ControllerGoal(forward_setpoint=0.0, sidemove_setpoint=0.0,
                             heading_setpoint=heading,
                             depth_setpoint=self.depth)
        self.controller.send_goal_and_wait(goal,
                                          execute_timeout=rospy.Duration(30),
                                          preempt_timeout=rospy.Duration(10))

        (heading, distance) = self.getPoints(req.x, req.y)
        goal = ControllerGoal(forward_setpoint=distance,
                              sidemove_setpoint=0.0, depth_setpoint=self.depth,
                              heading_setpoint=heading)
        self.controller.send_goal_and_wait(goal, execute_timeout=
                                          rospy.Duration(300),
                                          preempt_timeout=rospy.Duration(
                                                10
                                              ))

        #errx = req.x - self.cpos['x']
        #erry = req.y - self.cpos['y']

        ##if err positive we overshot
        ##if err negative we undershot

        #errx = errx * np.cos(self.heading)
        #erry = erry * np.sin(self.heading)

        #goal = ControllerGoal(forward_setpoint=errx, sidemove_setpoint=erry,
        #                     heading_setpoint=self.heading,
        #                     depth_setpoint=self.depth)
        #self.controller.send_goal_and_wait(goal,
        #                                  execute_timeout=rospy.Duration(15),
        #                                  preempt_timeout=rospy.Duration(10))

        #goal = ControllerGoal(forward_setpoint=0, sidemove_setpoint=0,
        #                     heading_setpoint=req.final_heading,
        #                     depth_setpoint=self.depth)
        #self.controller.send_goal_and_wait(goal,
        #                                   execute_timeout=rospy.Duration(30),
        #                                   preempt_timeout=rospy.Duration(10))
        #rospy.loginfo("Done navigatiing to (%s, %s), current (%s, %s)",
        #             str(req.x), str(req.y), str(self.cpos['x']),
        #             str(self.cpos['y']))
        return navigate2dResponse(status=True)

    def enable_PID(self, nav=False):
            self.initController(forward=True, sidemove=True,
                                roll=True, pitch=True, heading=True,
                                depth=True, navigation=False,topside=False)

    def getPoints(self, y, x):
        initial = [self.cpos['x'], self.cpos['y']]
        target = [x, y]
        magnitude = np.sqrt(sum(map(lambda k: k * k,
                                    list(i - j for i,
                                         j in zip(target, initial)))))
        n_vec = list((i - j) for i, j in zip([initial[0], initial[1] + 1],
                                             initial))
        dv = list((i - j) for i, j in zip(target, initial))
        mag_dv = np.sqrt(sum(map(lambda x: x * x, dv)))
        dotp = sum(list((i * j) for i, j in zip(n_vec, dv)))
        heading = np.rad2deg(np.arccos(dotp/mag_dv))
        if(target[0] < initial[0]):
            heading = 360.0 - heading
        return (heading, magnitude)


if __name__ == "__main__":
    rospy.init_node("navigate2D", anonymous=False)
    hook = Navigation()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("interrupted")
