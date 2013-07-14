from v4_mission_planner import *

if __name__ == '__main__':
    
    sm_mission = smach.StateMachine(outcomes=['mission_complete','mission_failed'])

    with sm_mission:
        smach.StateMachine.add('COUNTDOWN', Countdown(0), transitions={'succeeded':'START'})
        smach.StateMachine.add('START',Start(5,0.2,0),
                                transitions={'succeeded':'NAV_TO_GATE'})
        
        smach.StateMachine.add('NAV_TO_GATE', NavMoveBase(1,1,-9,-8.7,0.5,210), transitions={'succeeded':'LANE_GATE_TASK', 'failed':'SURFACE'})

        lane_gate = smach.StateMachine(outcomes=['lane_complete', 'lane_failed'])
        with lane_gate:
            smach.StateMachine.add('LANE_HOVER', HoverSearch('lane', 10, False, 1), transitions={'succeeded':'LANE_STORE', 'failed':'LANE_SEARCH'})
            smach.StateMachine.add('LANE_SEARCH', LinearSearch('lane', 20, -1, 'sway', False, 1), transitions={'succeeded':'LANE_STORE', 'failed':'LANE_SEARCH2'})
            smach.StateMachine.add('LANE_SEARCH2', LinearSearch('lane', 20, 2, 'sway', False, 1), transitions={'succeeded':'LANE_STORE', 'failed':'lane_failed'})
            smach.StateMachine.add('LANE_STORE', StoreGlobalCoord('mission_lane_gate'), transitions={'succeeded':'LANE_GATE'})
            smach.StateMachine.add('LANE_GATE', WaitOut('lane', 60), transitions={'succeeded':'LANE_HEADINGCHANGE', 'failed':'lane_failed'})
            smach.StateMachine.add('LANE_HEADINGCHANGE', GoToHeading(20), transitions={'succeeded':'lane_complete'})
        smach.StateMachine.add('LANE_GATE_TASK', lane_gate, transitions={'lane_complete':'TRAFFIC_TASK', 'lane_failed':'SURFACE'})        

        traffic = smach.StateMachine(outcomes=['traffic_complete', 'traffic_failed'])
        with traffic:
            smach.StateMachine.add('TRAFFIC_SEARCH', LinearSearch('traffic', 60, 1, 'fwd'), transitions={'succeeded':'TRAFFIC_STORE', 'failed':'TRAFFIC_SEARCH2'})
            smach.StateMachine.add('TRAFFIC_SEARCH2', LinearSearch('traffic', 60, -1, 'sway'), transitions={'succeeded':'TRAFFIC_STORE', 'failed':'TRAFFIC_SEARCH3'})
            smach.StateMachine.add('TRAFFIC_SEARCH3', LinearSearch('traffic', 60, 2, 'sway'), transitions={'succeeded':'TRAFFIC_STORE', 'failed':'traffic_failed'})
            smach.StateMachine.add('TRAFFIC_STORE', StoreGlobalCoord('mission_traffic'), transitions={'succeeded':'TRAFFIC'})
            smach.StateMachine.add('TRAFFIC', WaitOut('traffic', 180), transitions={'succeeded':'TRAFFIC_MOVETOSIDE', 'failed':'traffic_failed'})
            smach.StateMachine.add('TRAFFIC_MOVETOSIDE', GoToDistance(15,-0.75), transitions={'succeeded':'traffic_complete'})
        smach.StateMachine.add('TRAFFIC_TASK', traffic, transitions={'traffic_complete':'LANE_TRAFFIC_TASK', 'traffic_failed':'SURFACE'})            

