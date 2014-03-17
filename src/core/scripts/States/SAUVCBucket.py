import rospy, smach

class State(smach.State):
    transitions = {'pass' : 'SAUVCFlare', 'fail' : 'fail'}
    outcomes = ['pass', 'fail']

    def __init__(self, world):
        smach.State.__init__(self, outcomes=self.outcomes)
        self.world = world

    def execute(self, userdata):
        rospy.loginfo("Waiting for bucket service")
        self.world.bucketService.wait_for_service()
        rospy.loginfo("Got bucket service")
        
        while not rospy.is_shutdown():
            if self.world.bucketDone:
                return 'pass'
            if self.world.bucketFailed:
                return 'fail'

