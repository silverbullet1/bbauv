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
            smach.StateMachine.add('HOVER', HoverSearch('lane', 10, False, 1), transitions={'succeeded':'STORE', 'failed':'SEARCH'})
            smach.StateMachine.add('SEARCH', LinearSearch('lane', 20, -1, 'sway', False, 1), transitions={'succeeded':'STORE', 'failed':'SEARCH2'})
            smach.StateMachine.add('SEARCH2', LinearSearch('lane', 20, 2, 'sway', False, 1), transitions={'succeeded':'STORE', 'failed':'lane_failed'})
            smach.StateMachine.add('STORE', StoreGlobalCoord('mission_lane_gate'), transitions={'succeeded':'LANE_GATE'})
            smach.StateMachine.add('LANE_GATE', WaitOut('lane', 60), transitions={'succeeded':'HEADINGCHANGE', 'failed':'lane_failed'})
            smach.StateMachine.add('HEADINGCHANGE', GoToHeading(20), transitions={'succeeded':'lane_complete'})
        smach.StateMachine.add('LANE_GATE_TASK', lane_gate, transitions={'lane_complete':'TRAFFIC_TASK', 'lane_failed':'SURFACE'})        

        traffic = smach.StateMachine(outcomes=['traffic_complete', 'traffic_failed'])
        with traffic:
            smach.StateMachine.add('SEARCH', LinearSearch('traffic', 60, 1, 'fwd'), transitions={'succeeded':'STORE', 'failed':'SEARCH2'})
            smach.StateMachine.add('SEARCH2', LinearSearch('traffic', 60, -1, 'sway'), transitions={'succeeded':'STORE', 'failed':'SEARCH3'})
            smach.StateMachine.add('SEARCH3', LinearSearch('traffic', 60, 2, 'sway'), transitions={'succeeded':'STORE', 'failed':'traffic_failed'})
            smach.StateMachine.add('STORE', StoreGlobalCoord('mission_traffic'), transitions={'succeeded':'TRAFFIC'})
            smach.StateMachine.add('TRAFFIC', WaitOut('traffic', 180), transitions={'succeeded':'MOVETOSIDE', 'failed':'traffic_failed'})
            smach.StateMachine.add('MOVETOSIDE', GoToDistance(15,-0.75, 'sway'), transitions={'succeeded':'MOVEFWD'})
            smach.StateMachine.add('MOVEFWD', GoToDistance(15,1, 'fwd'), transitions={'succeeded':'MOVEFWD'})
        smach.StateMachine.add('TRAFFIC_TASK', traffic, transitions={'traffic_complete':'LANE_TRAFFIC_TASK', 'traffic_failed':'SURFACE'})            

        lane_traffic = smach.StateMachine(outcomes=['lane_complete', 'lane_failed'])
        with lane_traffic:
            smach.StateMachine.add('HOVER', HoverSearch('lane', 10, False, 1), transitions={'succeeded':'STORE', 'failed':'SEARCH'})
            smach.StateMachine.add('SEARCH', LinearSearch('lane', 20, 2, 'sway', False, 1), transitions={'succeeded':'STORE', 'failed':'SEARCH2'})
            smach.StateMachine.add('SEARCH2', LinearSearch('lane', 20, -1, 'sway', False, 1), transitions={'succeeded':'STORE', 'failed':'lane_failed'})
            smach.StateMachine.add('STORE', StoreGlobalCoord('mission_lane_traffic'), transitions={'succeeded':'LANE_GATE'})
            smach.StateMachine.add('LANE_GATE', WaitOut('lane', 60), transitions={'succeeded':'HEADINGCHANGE', 'failed':'lane_failed'})
            smach.StateMachine.add('HEADINGCHANGE', GoToHeading(20), transitions={'succeeded':'lane_complete'})
        smach.StateMachine.add('LANE_TRAFFIC_TASK', lane_gate, transitions={'lane_complete':'TOLL_TASK', 'lane_failed':'SURFACE'})

        toll = smach.StateMachine(outcomes=['toll_complete', 'toll_failed'])
        with toll:
            smach.StateMachine.add('DEPTHCHANGE', GoToDepth(10,), transitions={'succeeded':'    SEARCH'})
            smach.StateMachine.add('SEARCH', LinearSearch('tollbooth', 30, 4, 'fwd'), transitions={'succeeded':'STORE', 'failed':'TOLL_SEARCH2'})
            smach.StateMachine.add('SEARCH2', LinearSearch('tollbooth', 30, -2, 'sway'), transitions={'succeeded':'STORE', 'failed':'SEARCH3'})
            smach.StateMachine.add('SEARCH3', LinearSearch('tollbooth', 60, 4, 'sway'), transitions={'succeeded':'TOLL_STORE', 'failed':'SEARCH4'})
            smach.StateMachine.add('SEARCH4', LinearSearch('tollbooth', 30, 2, 'fwd'), transitions={'succeeded':'STORE', 'failed':'SEARCH5'})
            smach.StateMachine.add('SEARCH5', LinearSearch('tollbooth', 60, -4, 'sway'), transitions={'succeeded':'STORE', 'failed':'SEARCH6'})
            smach.StateMachine.add('SEARCH6', LinearSearch('tollbooth', 60, 4, 'sway'), transitions={'succeeded':'STORE', 'failed':'failed'})
            smach.StateMachine.add('STORE', StoreGlobalCoord('toll'), transitions={'succeeded':'TOLLBOOTH'})            
            smach.StateMachine.add('TOLLBOOTH', WaitOut('tollbooth', 180), transitions={'succeeded':'toll_complete', 'failed':'toll_failed'})
        smach.StateMachine.add('TOLL_TASK', toll, transitions={'toll_complete':'SPEED_TASK', 'toll_failed':'SPEED_TASK'})
   
