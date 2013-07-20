from v4_mission_planner import *

if __name__ == '__main__':

    sm_mission = smach.StateMachine(outcomes=['mission_complete','mission_failed'])

    with sm_mission:
        smach.StateMachine.add('COUNTDOWN', Countdown(0), transitions={'succeeded':'START'})
        smach.StateMachine.add('START',Start(5,0.5,90),transitions={'succeeded':'MOVERIGHT'})
        smach.StateMachine.add('MOVERIGHT', LinearSearch('speedtrap', 60, 3, 'sway'), transitions={'succeeded':'TASK', 'failed':'SURFACE_SADLY'})
        smach.StateMachine.add('TASK', WaitOut('speedtrap', 120), transitions={'succeeded':'SURFACE_VICTORIOUSLY', 'failed':'SURFACE_SADLY'})
        smach.StateMachine.add('SURFACE_SADLY', GoToDepth(20, 0.5), transitions={'succeeded':'mission_failed'})
        smach.StateMachine.add('SURFACE_VICTORIOUSLY', GoToDepth(5, 0.5), transitions={'succeeded':'VICTORY_SPIN'})
        smach.StateMachine.add('VICTORY_SPIN', GoToHeading(60, 0, relative=True), transitions={'succeeded':'mission_complete'})
        
    sm_mission = smach.StateMachine(outcomes=['mission_complete','mission_failed'])

    with sm_mission:
        smach.StateMachine.add('COUNTDOWN', Countdown(0), transitions={'succeeded':'START'})
        smach.StateMachine.add('START',Start(5,1,240),transitions={'succeeded':'HOVER'})
        smach.StateMachine.add('HOVER', HoverSearch('park', 60), transitions={'succeeded':'TASK', 'failed':'SURFACE_SADLY'})
        smach.StateMachine.add('TASK', WaitOut('park', 60), transitions={'succeeded':'SURFACE_VICTORIOUSLY', 'failed':'SURFACE_SADLY'})
        smach.StateMachine.add('SURFACE_SADLY', GoToDepth(20, 0.5), transitions={'succeeded':'mission_failed'})
        smach.StateMachine.add('SURFACE_VICTORIOUSLY', GoToDepth(5, 0.5), transitions={'succeeded':'VICTORY_SPIN'})
        smach.StateMachine.add('VICTORY_SPIN', GoToHeading(60, 0, relative=True), transitions={'succeeded':'mission_complete'})       
        
#    sm_mission = smach.StateMachine(outcomes=['mission_complete','mission_failed'])

    with sm_mission:
        smach.StateMachine.add('COUNTDOWN', Countdown(0), transitions={'succeeded':'START'})
        smach.StateMachine.add('START',Start(10,0.5,70),transitions={'succeeded':'GO_FWD'})
        smach.StateMachine.add('GO_FWD', GoToDistance(80, 11, 'fwd'), transitions={'succeeded':'HOVER'})
        smach.StateMachine.add('HOVER', HoverSearch('tollbooth', 30), transitions={'succeeded':'TOLLBOOTH', 'failed':'MOVERIGHT'})
        smach.StateMachine.add('TOLLBOOTH', WaitOut('tollbooth', 180), transitions={'succeeded':'RISE', 'failed': 'RISE'})
        smach.StateMachine.add('RISE', GoToDepth(10, 0.5), transitions={'succeeded':'FACEWALL'})
        smach.StateMachine.add('FACEWALL', GoToHeading(10, 90), transitions={'succeeded':'MOVERIGHT'})
        smach.StateMachine.add('MOVERIGHT', LinearSearch('speedtrap', 60, 4, 'sway'), transitions={'succeeded':'TASK', 'failed':'SURFACE_SADLY'})
        smach.StateMachine.add('TASK', WaitOut('speedtrap', 120), transitions={'succeeded':'SURFACE_VICTORIOUSLY', 'failed':'SURFACE_SADLY'})
        smach.StateMachine.add('SURFACE_SADLY', GoToDepth(20, 0.5), transitions={'succeeded':'mission_failed'})
        smach.StateMachine.add('SURFACE_VICTORIOUSLY', GoToDepth(5, 0.5), transitions={'succeeded':'VICTORY_SPIN'})
        smach.StateMachine.add('VICTORY_SPIN', GoToHeading(60, 0, relative=True), transitions={'succeeded':'mission_complete'})         