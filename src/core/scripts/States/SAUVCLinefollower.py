import rospy, smach, actionlib

from bbauv_msgs.srv import *
from bbauv_msgs.msg import *

class Linefollower(smach.State):
    def __init__(self, world):
        smach.State.__init__(self, outcomes=['pass', 'fail'])
        self.world = world
        self.lineDone = False
        self.bucketDone = False
        self.lineFailed = False
        self.bucketFailed = False
        self.world.fromLinefollower =\
        rospy.Service("/linefollower/vision_to_mission", vision_to_mission,
                      self.LinefollowerCallback)
        rospy.loginfo("LIENFOLLOWER SERVICE UP")
        self.world.toLinefollower =\
        rospy.ServiceProxy("/linefollower/mission_to_vision", mission_to_vision)

        self.world.fromBucket = rospy.Service("/bucket/vision_to_mission",
                                              vision_to_mission,
                                              self.bucketCallback)
        self.world.toBucket = rospy.ServiceProxy("/bucket/mission_to_vision",
                                                 mission_to_vision)

    def activateVisionNode(self):
        rospy.loginfo("Activating thien")
        self.world.toLinefollower.wait_for_service()
        rospy.loginfo("ended waiting for thien")
        r = self.world.toBucket(start_request=True, start_ctrl=controller(depth_setpoint=self.world.config['bucket_depth']), abort_request=False)
        rr = self.world.toLinefollower(start_request=True,
                                  start_ctrl=controller(depth_setpoint=self.world.config['linefollower_depth'],
                                                        heading_setpoint=self.world.static_yaw),
                                  abort_request=False)
        rospy.loginfo("Linefollower replied with %s" % (str(r)))
        rospy.loginfo("Bucket replied with %s" % (str(rr)))
        self.world.LinefollowerActive = True

    def bucketCallback(self, r):
        if r.search_request:
            resp = self.world.toLinefollower(start_request=False,
                                      start_ctrl=controller(depth_setpoint=self.world.config['bucket_depth'],
                                      heading_setpoint=self.world.yaw),
                                      abort_request=True)
            #self.world.flare_heading = resp.task_complete_ctrl.heading_setpoint
            print str(r)
            self.world.lights.publish(7)
            rospy.sleep(0.5)
            self.world.lights.publish(9)
            self.world.lastLineHeading = (self.world.currPos['x'],
                                          self.world.currPos['y'])
            self.lineDone = True
            self.world.LinefollowerActive = False
            self.world.flare_heading = self.world.yaw
            return vision_to_missionResponse(True, True,
                                             controller(heading_setpoint=float(self.world.yaw)))
        if r.task_complete_request:
            r = self.world.toBucket(start_request=True, start_ctrl=controller(), abort_request=True)
            print str(r)
            self.bucketDone = True
            return vision_to_missionResponse(True, True, controller())
        if r.fail_request:
            self.bucketFailed = True

    def LinefollowerCallback(self, req):
        if req.task_complete_request:
            self.world.toLinefollower(start_request=False,
                                      start_ctrl=controller(),
                                      abort_request=True)
            
            self.world.LinefollowerActive = False
            self.lineDone = True
            return vision_to_missionResponse(True, True, controller())
        if req.search_request:
            self.activateVisionNode()
            return vision_to_missionResponse(True, True, controller())
        if req.fail_request:
            self.world.LinefollowerActive = False
            self.lineFailed = True
            return vision_to_missionResponse(False, False, controller())

    def execute(self, userdata):
        self.world.lights.publish(2)
        rospy.sleep(0.5)
        self.world.lights.publish(9)
        self.activateVisionNode()
        while not rospy.is_shutdown():
            if self.lineDone and self.bucketDone:
                return 'pass'
            if self.lineFailed or self.bucketFailed:
                return 'fail'


class Flare(smach.State):
    def __init__(self, world):
        smach.State.__init__(self, outcomes=['pass', 'fail'])
        self.world = world

    def whack(self):
        sidemove = self.world.lastLineHeading[0] - self.world.currPos['x']
        forward = self.world.lastLineHeading[0] - self.world.currPos['y']
        goal = ControllerGoal(forward_setpoint=forward,
                              sidemove_setpoint=sidemove,
                              heading_setpoint=self.world.flare_heading,
                              depth_setpoint=self.world.config['sauvc_depth'])
        r = self.world.actionServer.send_goal_and_wait(goal,
                                                   execute_timeout=rospy.Duration(60))
        if r == actionlib.GoalStatus.SUCCEEDED:
            goal = ControllerGoal(forward_setpoint=5.0, sidemove_setpoint=0.0,
                                  heading_setpoint=self.world.flare_heading,
                                  depth_setpoint=self.world.config['sauvc_depth'])
            rr = self.world.actionServer.send_goal_and_wait(goal,
                                                            execute_timeout=rospy.Duration(60))
            if rr == actionlib.GoalStatus.SUCCEEDED:
                goal = ControllerGoal(forward_setpoint=0.0,
                                      sidemove_setpoint=0.0,
                                      heading_setpoint=self.world.yaw,
                                      depth_setpoint=0.0)
                rrr == self.world.actionServer.send_goal_and_wait(goal,
                                                           execute_timeout=rospy.Duration(20))
                if rrr == actionlib.GoalStatus.SUCCEEDED:
                    return 'pass'

        else:
            return 'fail'

    def execute(self, userdata):
        return self.whack()

class State(smach.State):
    outcomes = ['pass', 'fail']
    transitions = {'pass' : 'pass', 'fail' : 'fail'}

    def __init__(self, world):
        smach.State.__init__(self, outcomes=self.outcomes)
        self.world = world
        self.world.LinefollowerActive = False
        self.world.BucketActive = False
        self.sm = smach.StateMachine(outcomes = self.outcomes)

    def addState(self):
        with self.sm:
            smach.StateMachine.add('LINEBUCKET', Linefollower(self.world),
                                   transitions={'pass' : 'FLARE', 'fail' : 'fail'})
            smach.StateMachine.add('FLARE', Flare(self.world),
                                   transitions={'pass' : 'pass', 'fail' : 'fail'})

    def execute(self, userdata):
        self.addState()
        outcome = self.sm.execute()
        return outcome
