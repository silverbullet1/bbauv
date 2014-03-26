import rospy, smach
from bbauv_msgs.msg import controller

class State(smach.State):
    transitions = {'pass' : 'SAUVCBucket', 'fail' : 'SAUVCAcoustics'}
    outcomes = ['pass', 'fail']

    def __init__(self, world):
        smach.State.__init__(self, outcomes=self.outcomes)
        self.world = world
        self.timeout = self.world.config["max_linefollower_time"]

    def timerCallback(self, e):
        rospy.logerr("Linefollower exceeded maximum time allowed. Aborting")
        self.world.linefollowerService(abort_request=True, start_request=False,
                                       start_ctrl=controller())
        rospy.loginfo("Got a reply from linefollower in timerCallback")
        rospy.logerr("killing bucket")
        try:
            self.world.bucketService(abort_request=True, start_request=False,
                                    start_ctrl=controller())
        except Exception, e:
            rospy.logerr("Exception killing bucket: %s" % (str(e)))
        finally:
            self.world.LinefollowerFailed = True


    def execute(self, userdata):
        self.world.lights.publish(3)
        self.timer = rospy.Timer(rospy.Duration(self.timeout),
                                    self.timerCallback, oneshot=True)
        tn = rospy.get_time()
        rospy.loginfo("Ros time now is %f, starting linefollower, %fs max" %
                      (tn, self.timeout + self.world.grace))
        rospy.loginfo("Waiting for linefollower service")
        self.world.linefollowerService.wait_for_service()
        rospy.loginfo("Got linefollower service")
        rospy.loginfo("Waiting for bucket service")
        self.world.bucketService.wait_for_service()
        rospy.loginfo("Got bucket service")

        self.world.activateLinefollower()
        self.world.activateBucket()
        while not rospy.is_shutdown():
            if self.world.linefollowerDone:
                self.timer.shutdown()
                self.world.grace = self.timeout - (rospy.get_time() - tn)
                return 'pass'
            if self.world.LinefollowerFailed:
                self.timer.shutdown()
                self.world.linefollowerDone = True
                self.world.grace = self.timeout - (rospy.get_time() - tn)
                return 'fail'
            if not self.world.linefollowerDone and not self.world.linefollowerActive:
                rospy.logerr("This should never happen")
                return 'fail'
        return 'fail'
