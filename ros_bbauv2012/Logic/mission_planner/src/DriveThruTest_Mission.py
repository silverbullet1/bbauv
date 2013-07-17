from v4_mission_planner import *

if __name__ == '__main__':
    
    sm_mission = smach.StateMachine(outcomes=['mission_complete','mission_failed'])

    with sm_mission:
        smach.StateMachine.add('COUNTDOWN', Countdown(0), transitions={'succeeded':'START'})
        smach.StateMachine.add('START',Start(20,0.5,90),transitions={'succeeded':'HOVER'})
        smach.StateMachine.add('HOVER', HoverSearch('acoustic', 30, start_depth=0.4, start_heading=90), transitions={'succeeded':'TASK', 'failed':'GOFWD'})
        smach.StateMachine.add('GOFWD', GoToDistance(30, 4, 'fwd'), transitions={'succeeded':'HOVER2'})
        smach.StateMachine.add('HOVER2', HoverSearch('acoustic', 30, start_depth=0.4, start_heading=90), transitions={'succeeded':'TASK', 'failed':'GOFWD_AGAIN'})
        smach.StateMachine.add('GOFWD_AGAIN', GoToDistance(30, 3, 'fwd'), transitions={'succeeded':'HOVER3'})
        smach.StateMachine.add('HOVER3', HoverSearch('acoustic', 30, start_depth=0.4, start_heading=90), transitions={'succeeded':'TASK', 'failed':'SURFACE_SADLY'})
        smach.StateMachine.add('TASK', WaitOut('acoustic', 180), transitions={'succeeded':'HOVER4', 'failed':'SURFACE_SADLY'})
        smach.StateMachine.add('HOVER4', HoverSearch('drivethru', 30), transitions={'succeeded':'TASK2', 'failed':'SEARCH_REAR'})
        smach.StateMachine.add('SEARCH_REAR', LinearSearch('drivethru', 30, -2, 'fwd'), transitions={'succeeded':'TASK2', 'failed':'SEARCH_FRONT'})
        smach.StateMachine.add('SEARCH_FRONT', LinearSearch('drivethru', 30, 4, 'fwd'), transitions={'succeeded':'TASK2', 'failed':'SURFACE_SADLY'})
        smach.StateMachine.add('TASK2', WaitOut('drivethru', 180), transitions={'succeeded':'SURFACE_VICTORIOUSLY', 'failed':'SURFACE_SADLY'})                
        smach.StateMachine.add('SURFACE_SADLY', GoToDepth(20, 0.5), transitions={'succeeded':'mission_failed'})
        smach.StateMachine.add('SURFACE_VICTORIOUSLY', GoToDepth(5, 0.5), transitions={'succeeded':'VICTORY_SPIN'})
        smach.StateMachine.add('VICTORY_SPIN', GoToHeading(60, 0, relative=True), transitions={'succeeded':'mission_complete'})
