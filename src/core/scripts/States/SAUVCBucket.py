import rospy, smach
from bbauv_msgs.msg import controller

class State(smach.State):
    transitions = {'pass' : 'SAUVCFlare', 'fail' : 'fail'}
    outcomes = ['pass', 'fail']

    def __init__(self, world):
        smach.State.__init__(self, outcomes=self.outcomes)
        self.world = world
        self.timeout = self.world.config["max_bucket_time"]
        self.timer = rospy.Timer(rospy.Duration(
            self.timeout + self.world.grace), self.timerCallback, oneshot=True)

    def timerCallback(self, e):
        rospy.logerr("Bucket detector ran out of time.")
        self.world.bucketService(abort_request=True, start_request=False,
                                 start_ctrl=controller())
        self.world.bucketDone = True

    def execute(self, userdata):
        tn = rospy.get_time()
        rospy.loginfo("Ros time now is %f, bucket has %fs." % (tn,
                                    self.timeout + self.world.grace))
        rospy.loginfo("Time carried on was %fs" % (self.world.grace))
        rospy.loginfo("Waiting for bucket service")
        self.world.bucketService.wait_for_service()
        rospy.loginfo("Got bucket service")
        
        while not rospy.is_shutdown():
            if self.world.bucketDone:
                self.timer.shutdown()
                self.world.grace = self.timeout - (rospy.get_time() - tn)
                return 'pass'
            if self.world.bucketFailed:
                self.world.grace = self.timeout - (rospy.get_time() - tn)
                return 'fail'

