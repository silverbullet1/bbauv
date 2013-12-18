from v4_mission_planner import *

if __name__ == '__main__':
    
    sm_mission = smach.StateMachine(outcomes=['mission_complete'])

    with sm_mission:
        
        smach.StateMachine.add('COUNTDOWN', Countdown(0), transitions={'succeeded':'START'})
        smach.StateMachine.add('START',Start(10,0.2,0),
                                transitions={'succeeded':'MARKER1'})
        
        smach.StateMachine.add('MARKER1', StoreGlobalCoord('mission_marker1'), transitions={'succeeded':'MOVE_LEFT'})
        smach.StateMachine.add('MOVE_LEFT', GoToDistance(30, 2, 'sway'), transitions={'succeeded':'MARKER2'})
        smach.StateMachine.add('MARKER2', StoreGlobalCoord('mission_marker2'), transitions={'succeeded':'MOVE_FWD'})
        smach.StateMachine.add('MOVE_FWD', GoToDistance(30, 2,'fwd'), transitions={'succeeded':'MARKER3'})
        smach.StateMachine.add('MARKER3', StoreGlobalCoord('mission_marker3'), transitions={'succeeded':'mission_complete'})
        
        
        