from v4_mission_planner import *

if __name__ == '__main__':

    sm_mission = smach.StateMachine(outcomes=['mission_complete','mission_failed'])

    with sm_mission:
        smach.StateMachine.add('COUNTDOWN', Countdown(0), transitions={'succeeded':'START'})
        smach.StateMachine.add('START',Start(5,0.5,0),transitions={'succeeded':'NAV_TO_GATE'})    
#         smach.StateMachine.add('TURN_TO_GATE', GoToHeading(10, 70), transitions={'succeeded':'GO_TO_GATE'})
#         smach.StateMachine.add('GO_TO_GATE', GoToDistance(30, 6, 'fwd'), transitions={'succeeded':'SURFACE_VICTORIOUSLY'})
        smach.StateMachine.add('NAV_TO_GATE', NavMoveBase(1,20,4.2,4.2,0.5,0), transitions={'succeeded':'SURFACE_VICTORIOUSLY', 'failed':'SURFACE_SADLY'})
        smach.StateMachine.add('SURFACE_SADLY', GoToDepth(20, 0.5), transitions={'succeeded':'mission_failed'})
        smach.StateMachine.add('SURFACE_VICTORIOUSLY', GoToDepth(5, 0.5), transitions={'succeeded':'VICTORY_SPIN'})
        smach.StateMachine.add('VICTORY_SPIN', GoToHeading(60, 0, relative=True), transitions={'succeeded':'mission_complete'})        
        