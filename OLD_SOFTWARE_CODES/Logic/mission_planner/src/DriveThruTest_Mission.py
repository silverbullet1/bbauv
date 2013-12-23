from v4_mission_planner import *

if __name__ == '__main__':
    
    sm_mission = smach.StateMachine(outcomes=['mission_complete','mission_failed'])

    with sm_mission:
        smach.StateMachine.add('COUNTDOWN', Countdown(0), transitions={'succeeded':'START'})
        smach.StateMachine.add('START',Start(10,0.5,70),transitions={'succeeded':'GOFWD'})
        smach.StateMachine.add('HOVER', HoverSearch('acoustic', 30, start_depth=0.5, start_heading=70), transitions={'succeeded':'TASK', 'failed':'GOFWD'})
        smach.StateMachine.add('GOFWD', GoToDistance(40, 1, 'fwd'), transitions={'succeeded':'HOVER2'})
        smach.StateMachine.add('HOVER2', HoverSearch('acoustic', 30, start_depth=0.5, start_heading=70), transitions={'succeeded':'TASK', 'failed':'GOFWD2'})
        smach.StateMachine.add('GOFWD2', GoToDistance(40, 2, 'fwd'), transitions={'succeeded':'HOVER3'})
        smach.StateMachine.add('HOVER3', HoverSearch('acoustic', 30, start_depth=0.5, start_heading=70), transitions={'succeeded':'TASK', 'failed':'GOFWD3'})
        smach.StateMachine.add('GOFWD3', GoToDistance(40, 2, 'fwd'), transitions={'succeeded':'HOVER4'})
        smach.StateMachine.add('HOVER4', HoverSearch('acoustic', 30, start_depth=0.5, start_heading=70), transitions={'succeeded':'TASK', 'failed':'SURFACE_SADLY'})

        smach.StateMachine.add('TASK', WaitOutAndSearch('acoustic','drivethru', 180), transitions={'task_succeeded':'HOVER5', 'search_succeeded':'TASK2','failed':'SURFACE_SADLY'})

        smach.StateMachine.add('TASK2', WaitOut('drivethru', 180), transitions={'succeeded':'SURFACE_VICTORIOUSLY', 'failed':'SURFACE_SADLY'})

        smach.StateMachine.add('HOVER5', HoverSearch('drivethru', 120), transitions={'succeeded':'TASK2', 'failed':'SEARCH_LEFT'})    
        smach.StateMachine.add('SEARCH_LEFT', LinearSearch('drivethru', 30, -1, 'sway'), transitions={'succeeded':'TASK2', 'failed':'SEARCH_RIGHT'})
        smach.StateMachine.add('SEARCH_RIGHT', LinearSearch('drivethru', 30, 2, 'sway'), transitions={'succeeded':'TASK2', 'failed':'SURFACE_SADLY'})
       
        smach.StateMachine.add('SURFACE_SADLY', GoToDepth(20, 0.5), transitions={'succeeded':'mission_failed'})
        smach.StateMachine.add('SURFACE_VICTORIOUSLY', GoToDepth(5, 0.5), transitions={'succeeded':'VICTORY_SPIN'})
        smach.StateMachine.add('VICTORY_SPIN', GoToHeading(60, 0, relative=True), transitions={'succeeded':'mission_complete'})

    sm_mission = smach.StateMachine(outcomes=['mission_complete','mission_failed'])

    with sm_mission:
        smach.StateMachine.add('COUNTDOWN', Countdown(0), transitions={'succeeded':'START'})
        smach.StateMachine.add('START',Start(5,0.5,100),transitions={'succeeded':'GOFWD'})
#        smach.StateMachine.add('HOVER', HoverSearch('acoustic', 30, start_depth=0.5, start_heading=70), transitions={'succeeded':'TASK', 'failed':'GOFWD'})

        smach.StateMachine.add('GOFWD', GoToDistance(20, 1, 'fwd'), transitions={'succeeded':'HOVER2'})
        smach.StateMachine.add('HOVER2', HoverSearch('acoustic', 60, start_depth=0.5, start_heading=100), transitions={'succeeded':'TASK', 'failed':'GOFWD2'})

        smach.StateMachine.add('GOFWD2', GoToDistance(20, 1, 'fwd'), transitions={'succeeded':'HOVER3'})
        smach.StateMachine.add('HOVER3', HoverSearch('acoustic', 60, start_depth=0.5, start_heading=100), transitions={'succeeded':'TASK', 'failed':'GOFWD3'})

        smach.StateMachine.add('GOFWD3', GoToDistance(20, 1, 'fwd'), transitions={'succeeded':'HOVER4'})
        smach.StateMachine.add('HOVER4', HoverSearch('acoustic', 60, start_depth=0.5, start_heading=100), transitions={'succeeded':'TASK', 'failed':'HOME'})
        
        smach.StateMachine.add('TASK', WaitOut('acoustic', 180), transitions={'succeeded':'NAV2', 'failed':'HOME'})
        
        smach.StateMachine.add('NAV2', Nav(30,60,30,-2,2,0.5,0.5,70), transitions={'succeeded':'HOVER5', 'failed':'HOME'})
        smach.StateMachine.add('HOVER5', HoverSearch('acoustic', 60), transitions={'succeeded':'TASK2', 'failed':'HOME'})
        smach.StateMachine.add('TASK2', WaitOut('acoustic', 180), transitions={'succeeded':'HOME', 'failed':'HOME'})
        smach.StateMachine.add('HOME', Nav(30,60,30,-0.25,0.25,0.5,0.5,0), transitions={'succeeded':'SURFACE', 'failed':'SURFACE'})
        smach.StateMachine.add('SURFACE', GoToDepth(5, 0.26, surface=True), transitions={'succeeded':'mission_complete'})