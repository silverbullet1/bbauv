import rospy, smach
from bbauv_msgs.msg import controller
from bbauv_msgs.srv import mission_to_vision

class Lane(smach.State):
    def __init__(self, world):
        smach.State.__init__(self, outcomes=['pass', 'fail'])
        self.world = world

    def execute(self, userdata):
        self.world.laneStatus['numMarkers'] += 1;
        if self.world.laneStatus['numMarkers'] > 5:
            self.world.laneStatus['numMarkers'] = 1;
        self.world.laneClient = rospy.ServiceProxy("/lane/mission_to_vision",
                                                  mission_to_vision)
        if self.world.laneStatus['heading'] is None:
            self.world.laneStatus['heading'] = self.world.current_yaw
        rospy.loginfo("Waiting for lane server");
        self.world.laneClient.wait_for_service()
        rospy.loginfo("Done waiting for laen server")
        self.world.enable_PID()
        if(self.world.laneStatus['numMarkers'] == 5):
            nLanes = 2;
        else:
            nLanes = 1;
        rospy.loginfo("Number of markers: %s", str(
                nLanes
            ))
        self.world.laneClient(start_request=True, abort_request=False,
                             start_ctrl=controller(
                                    depth_setpoint=0.3,
                                    heading_setpoint=
                                 self.world.laneStatus['heading'],
                                 )
                              , numLanes = nLanes, chosenLane = 1
                             )
        self.world.laneStatus['active'] = True
        self.world.laneStatus['complete'] = False
        self.world.laneStatus['aborted'] = False
        rospy.loginfo("status of lane: %s", str(
                self.world.laneStatus['complete']
            ))

        while not rospy.is_shutdown():
            if self.world.laneStatus['complete']:
                return 'pass'
            if self.world.laneStatus['aborted']:
                return 'fail'

class State(smach.State):
    transitions = {'pass' : 'RGB', 'fail' : 'fail'}
    outcomes = ['pass', 'fail']

    def __init__(self, world):
        smach.State.__init__(self, outcomes=self.outcomes)
        self.world = world
        self.world.startLaneServer()
        self.sm = smach.StateMachine(outcomes=self.outcomes)
        with self.sm:
            smach.StateMachine.add('LANE', Lane(self.world),
                                   transitions={'pass':'pass',
                                                'fail':'fail'})

    def execute(self, userdata):
        outcome = self.sm.execute()
        self.world.controller.cancel_all_goals()
        return outcome
