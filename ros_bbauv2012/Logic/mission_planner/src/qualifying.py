from v4_mission_planner import *

if __name__ == '__main__':
    
    sm_mission = StateMachine(outcomes=['mission_complete','mission_failed'])

    with sm_mission:
        StateMachine.add('COUNTDOWN', Countdown(20), transitions={'succeeded':'START'})
        StateMachine.add('START',Start(5,0.7,40),transitions={'succeeded':'TURN_TO_GATE'})    
        StateMachine.add('TURN_TO_GATE', GoToHeading(10, 40), transitions={'succeeded':'GO_TO_GATE'})
        StateMachine.add('GO_TO_GATE', GoToDistance(60, 20, 'fwd'), transitions={'succeeded':'SURFACE_HAPPY'})
#        StateMachine.add('WYPT1', Nav(30,60,30, 10.73,16.53, 0.6,0.6,60), transitions={'succeeded':'SURFACE_HAPPY', 'failed':'SURFACE_SAD'})
        StateMachine.add('SURFACE_HAPPY', GoToDepth(5, 0.26), transitions={'succeeded':'mission_complete'})
#        StateMachine.add('SURFACE_SAD', GoToDepth(5, 0.26), transitions={'succeeded':'mission_failed'})


    
