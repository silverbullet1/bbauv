from v4_mission_planner import *

if __name__ == '__main__':

    sm_mission = smach.StateMachine(outcomes=['mission_complete','mission_failed'])

    with sm_mission:
        smach.StateMachine.add('COUNTDOWN', Countdown(700), transitions={'succeeded':'START'})
        smach.StateMachine.add('START',Start(15,0.2,90),transitions={'succeeded':'HOVER'})
        smach.StateMachine.add('HOVER', HoverSearch('drivethru', 30), transitions={'succeeded':'DRIVE_THRU', 'failed':'SURFACE_SADLY'})
        smach.StateMachine.add('DRIVE_THRU', WaitOut('drivethru', 60), transitions={'succeeded':'SURFACE_VICTORIOUSLY', 'failed':'SURFACE_SADLY'})
        smach.StateMachine.add('SURFACE_SADLY', GoToDepth(20, 0, surface=True), transitions={'succeeded':'mission_failed'})
        smach.StateMachine.add('SURFACE_VICTORIOUSLY', GoToDepth(5, 0, surface=True), transitions={'succeeded':'VICTORY_SPIN'})
        smach.StateMachine.add('VICTORY_SPIN', GoToHeading(60, 300, relative=True), transitions={'succeeded':'mission_complete'})
        
        