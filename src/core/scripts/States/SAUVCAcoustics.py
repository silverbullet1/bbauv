import rospy, smach
from bbauv_msgs.srv import navigate2d
from bbauv_msgs.msg import controller, ControllerGoal

class State(smach.State):
    transitions = {'pass' : 'pass', 'fail' : 'fail'}
    outcomes = ['pass', 'fail']

    def __init__(self, world):
        smach.State.__init__(self, outcomes=self.outcomes)
        self.world = world
        
    def execute(self, userdata):
        self.world.lights.publish(8)
        rospy.loginfo("Surfacing to 0.5m for DVL accuracy.")
        goal = ControllerGoal(forward_setpoint=0.0, sidemove_setpoint=0.0,
                              depth_setpoint=0.5,
                              heading_setpoint=self.world.current_yaw)
        rospy.loginfo("Waiting for locomotionServer")
        self.world.locomotionServer.wait_for_server()
        rospy.loginfo("Got it")
        self.world.locomotionServer.send_goal_and_wait(goal)
        rospy.loginfo("Locomotion done")
        nav2d = rospy.ServiceProxy("/navigate2D", navigate2d)
        rospy.loginfo("Waiting for navigate2d")
        nav2d.wait_for_service()
        rospy.loginfo("Got nav2d")
        r = nav2d(6.5, -7.5)
        rospy.loginfo("Waiting for acoustics service")
        self.world.acousticService.wait_for_service()
        rospy.loginfo("Got acoustics service, executing")
        self.world.acousticService(
            start_request=True, abort_request=False,
                start_ctrl=controller())
        while not rospy.is_shutdown():
            if self.world.acousticsDone:
                return 'pass'
            if self.world.acousticsFailed:
                return 'fail'
#    def execute(self, userdata):
#        self.world.lights.publish(9)
#        self.world.lights.publish(8)
#        goal = ControllerGoal(depth_setpoint=0.6,
#                              heading_setpoint=(self.world.current_yaw + 180.0) % 360,
#                              sidemove_setpoint=0.0, forward_setpoint=0)
#        self.world.locomotionServer.wait_for_server()
#        self.world.locomotionServer.send_goal_and_wait(goal,
#                                                       execute_timeout=rospy.Duration(30))
#        goal = ControllerGoal(depth_setpoint=0.6,
#                              heading_setpoint=self.world.current_yaw,
#                              sidemove_setpoint=0.0, forward_setpoint=5.0)
#        self.world.locomotionServer.send_goal_and_wait(goal,
#                                                       execute_timeout=rospy.Duration(60))
#        self.world.acousticService(start_request=True, abort_request=False,
#                                   start_ctrl=controller())
#        while not rospy.is_shutdown():
#            if self.world.acousticsDone:
#                return 'pass'
#            if self.world.acousticsFailed:
#                return 'fail'
