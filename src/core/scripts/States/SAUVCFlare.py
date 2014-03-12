import rospy, smach
from bbauv_msgs.msg import controller

class State(smach.State):
    transitions = {'pass' : 'pass', 'fail' : 'fail'}
    outcomes = ['pass', 'fail']

    def __init__(self, world):
        smach.State.__init__(self, outcomes=self.outcomes)
        self.world = world
        self.timeout = self.world.config["max_flare_timeout"]

        self.startpos = self.world.current_pos

    def timerCallback(self, e):
        rospy.logerr("Flare ran out of time")
        r = self.world.flareService(abort_request=True, start_request=False,
                                start_ctrl=controller())
        rospy.loginfo("Flare replied to abort request: %s" % (str(r)))
        self.world.flareFailed = True

    def execute(self, userdata):
        self.timer = rospy.Timer(rospy.Duration(
            self.timeout + self.world.grace), self.timerCallback, oneshot=True)
        tn = rospy.get_time()
        rospy.loginfo("Ros time now is %f, flare has %fs" % (tn,
                                    self.timeout + self.world.grace))
        rospy.loginfo("Amount of grace time carried over is %f" %
                      self.world.grace)
        rospy.loginfo("Waiting for flare service")
        try:
            self.world.flareService.wait_for_service(timeout=10)
        except Exception, e:
            rospy.logerr("waiting for flare service timed out: %s" % (str(e)))
            return 'fail'
        rospy.loginfo("Got flare service, starting")
        self.world.flareService(
            start_request=True, abort_request=False,
            start_ctrl=controller(depth_setpoint=self.world.config['sauvc_depth'],
                                  heading_setpoint=self.world.current_yaw)
            )
        while not rospy.is_shutdown():
            if self.world.flareDone:
                self.world.grace = self.timeout - (rospy.get_time() - tn)
                return 'pass'
            if self.world.flareFailed:
                self.world.grace = self.timeout - (rospy.get_time() - tn)
                return 'fail'

