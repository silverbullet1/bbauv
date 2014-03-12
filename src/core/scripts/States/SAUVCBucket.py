import rospy, smach
from bbauv_msgs.msg import controller
from planar import BoundingBox

class State(smach.State):
    transitions = {'pass' : 'SAUVCFlare', 'fail' : 'SAUVCFlare'}
    outcomes = ['pass', 'fail']

    def __init__(self, world):
        smach.State.__init__(self, outcomes=self.outcomes)
        self.world = world
        self.timeout = self.world.config["max_bucket_time"]

    def timerCallback(self, e):
        rospy.logerr("Bucket detector ran out of time.")
        rospy.logerr("Sending bucket abort request because of time")
        r = self.world.bucketService(abort_request=True, start_request=False,
                                 start_ctrl=controller())
        rospy.loginfo("Bucket replied to the abort signal with: %s" % (str(r)))
        self.world.bucketDone = True

    def execute(self, userdata):
        self.bbox = BoundingBox([(self.world.current_pos['x'] - 1,
                                  self.world.current_pos['y'] - 1),
                                 (self.world.current_pos['x'] + 1,
                                  self.world.current_pos['y'] + 1)])
        self.timer = rospy.Timer(rospy.Duration(
                    self.timeout), self.timerCallback, oneshot=True)
        tn = rospy.get_time()
        rospy.loginfo("Ros time now is %f, bucket has %fs." % (tn,
                                    self.timeout))
        rospy.loginfo("Time carried on was %fs" % (self.world.grace))
        rospy.loginfo("Waiting for bucket service")
        self.world.bucketService.wait_for_service()
        rospy.loginfo("Got bucket service")
        
        while not rospy.is_shutdown():
            if not self.bbox.contains_point((self.world.current_pos['x'],
                                             self.world.current_pos['y'])):
                rospy.logerr("Bucket detector went out of the bounding box")
                rospy.logerr("Sending bucket abort signal because of bounding\
                             box")
                r = self.world.bucketService(abort_request=True,
                                         start_request=False,
                                         start_ctrl=controller())
                if r.abort_response:
                    self.world.bucketFailed = True
            if self.world.bucketDone:
                self.timer.shutdown()
                return 'pass'
            if self.world.bucketFailed:
                return 'fail'

