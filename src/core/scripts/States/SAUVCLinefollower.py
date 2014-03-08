import rospy, smach

class State(smach.State):
    transitions = {'pass' : 'SAUVCBucket', 'fail' : 'fail'}
    outcomes = ['pass', 'fail']

    def __init__(self, world):
        smach.State.__init__(self, outcomes=self.outcomes)
        self.world = world

    def execute(self, userdata):
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
                return 'pass'
            if self.world.LinefollowerFailed:
                self.world.linefollowerDone = True
                return 'fail'
            if not self.world.linefollowerDone and not self.world.linefollowerActive:
                return 'fail'
        return 'fail'
