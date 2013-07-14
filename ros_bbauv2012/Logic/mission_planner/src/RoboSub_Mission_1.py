from v4_mission_planner import *

if __name__ == '__main__':

    sm_mission = smach.StateMachine(outcomes=['mission_complete','mission_failed'])

    with sm_mission:
        smach.StateMachine.add('COUNTDOWN', Countdown(0), transitions={'succeeded':'START'})
        smach.StateMachine.add('START',Start(1,0.5,210),
                                transitions={'succeeded':'NAV_TO_GATE'})
        
        smach.StateMachine.add('NAV_TO_GATE', NavMoveBase(1,1,-9,-8.7,0.5,210), transitions={'succeeded':'LANE_GATE_TASK', 'failed':'SURFACE'})
        #Starting right above lane1
#         smach.StateMachine.add('NAV_TO_GATE', NavMoveBase(0.5,0.5,-9,-8.7,0.5,235), transitions={'succeeded':'LANE_GATE_TASK', 'failed':'SURFACE'})
        
        lane_gate = smach.StateMachine(outcomes=['lane_complete', 'lane_failed'])
        with lane_gate:
            smach.StateMachine.add('LANE_HOVER', HoverSearch('lane', 10, False, 1), transitions={'succeeded':'LANE_STORE', 'failed':'LANE_SEARCH'})
            smach.StateMachine.add('LANE_SEARCH', LinearSearch('lane', 20, -1.5, 'sway', False, 1), transitions={'succeeded':'LANE_STORE', 'failed':'LANE_SEARCH2'})
            smach.StateMachine.add('LANE_SEARCH2', LinearSearch('lane', 20, -2, 'fwd', False, 1), transitions={'succeeded':'LANE_STORE', 'failed':'LANE_SEARCH3'})
            smach.StateMachine.add('LANE_SEARCH3', LinearSearch('lane', 20, 3, 'sway', False, 1), transitions={'succeeded':'LANE_STORE', 'failed':'lane_failed'})
            smach.StateMachine.add('LANE_STORE', StoreGlobalCoord('mission_lane_gate'), transitions={'succeeded':'LANE_GATE'})
            smach.StateMachine.add('LANE_GATE', WaitOut('lane', 60), transitions={'succeeded':'LANE_HEADINGCHANGE', 'failed':'lane_failed'})
            smach.StateMachine.add('LANE_HEADINGCHANGE', GoToHeading(10), transitions={'succeeded':'lane_complete'})
        smach.StateMachine.add('LANE_GATE_TASK', lane_gate, transitions={'lane_complete':'TRAFFIC_TASK', 'lane_failed':'SURFACE'})        

        traffic = smach.StateMachine(outcomes=['traffic_complete', 'traffic_failed'])
        with traffic:
            smach.StateMachine.add('TRAFFIC_DEPTHCHANGE', GoToDepth(15,2), transitions={'succeeded':'TRAFFIC_SEARCH'})
            smach.StateMachine.add('TRAFFIC_SEARCH', LinearSearch('traffic', 60, 2, 'fwd'), transitions={'succeeded':'TRAFFIC_STORE', 'failed':'TRAFFIC_SEARCH2'})
            smach.StateMachine.add('TRAFFIC_SEARCH2', LinearSearch('traffic', 60, -1, 'sway'), transitions={'succeeded':'TRAFFIC_STORE', 'failed':'TRAFFIC_SEARCH3'})
            smach.StateMachine.add('TRAFFIC_SEARCH3', LinearSearch('traffic', 60, 2, 'sway'), transitions={'succeeded':'TRAFFIC_STORE', 'failed':'traffic_failed'})
            smach.StateMachine.add('TRAFFIC_STORE', StoreGlobalCoord('mission_traffic'), transitions={'succeeded':'TRAFFIC'})
            smach.StateMachine.add('TRAFFIC', WaitOut('traffic', 180), transitions={'succeeded':'TRAFFIC_DEPTHCHANGE2', 'failed':'traffic_failed'})
            smach.StateMachine.add('TRAFFIC_DEPTHCHANGE2', GoToDepth(15,0.5), transitions={'succeeded':'traffic_complete'})
        smach.StateMachine.add('TRAFFIC_TASK', traffic, transitions={'traffic_complete':'LANE_TRAFFIC_TASK', 'traffic_failed':'SURFACE'})            

        lane_traffic = smach.StateMachine(outcomes=['lane_complete', 'lane_failed'])
        with lane_traffic:
            smach.StateMachine.add('LANE_FORWARD', GoToDistance(30, 1.5, 'fwd'), transitions={'succeeded':'LANE_HOVER'})
            smach.StateMachine.add('LANE_HOVER', HoverSearch('lane', 10, False, 1), transitions={'succeeded':'LANE_STORE', 'failed':'LANE_SEARCH'})
            smach.StateMachine.add('LANE_SEARCH', LinearSearch('lane', 60, 2, 'sway', False, 1), transitions={'succeeded':'LANE_STORE', 'failed':'LANE_SEARCH2'})
            smach.StateMachine.add('LANE_SEARCH2', LinearSearch('lane', 60, -4, 'sway', False, 1), transitions={'succeeded':'LANE_STORE', 'failed':'lane_failed'})
            smach.StateMachine.add('LANE_STORE', StoreGlobalCoord('mission_lane_traffic'), transitions={'succeeded':'LANE_TRAFFIC'})
            smach.StateMachine.add('LANE_TRAFFIC', WaitOut('lane', 60), transitions={'succeeded':'LANE_HEADINGCHANGE', 'failed':'lane_failed'})
            smach.StateMachine.add('LANE_HEADINGCHANGE', GoToHeading(10), transitions={'succeeded':'lane_complete'})
#            smach.StateMachine.add('LANE_ABOUTURN', GoToHeading(10, 180, True), transitions={'succeeded':'lane_complete'})      
        smach.StateMachine.add('LANE_TRAFFIC_TASK', lane_traffic, transitions={'lane_complete':'PARK_TASK', 'lane_failed':'PARK_TASK'})  

        park = smach.StateMachine(outcomes=['park_complete', 'park_failed'])
        with park:
            smach.StateMachine.add('PARK_DEPTHCHANGE', GoToDepth(10,2), transitions={'succeeded':'PARK_SEARCH'})
            smach.StateMachine.add('PARK_SEARCH', LinearSearch('park', 60, 1, 'fwd'), transitions={'succeeded':'PARK_STORE', 'failed':'PARK_SEARCH2'})
            smach.StateMachine.add('PARK_SEARCH2', LinearSearch('park', 60, 4, 'sway'), transitions={'succeeded':'PARK_STORE', 'failed':'park_failed'})
            smach.StateMachine.add('PARK_STORE', StoreGlobalCoord('mission_park1'), transitions={'succeeded':'PARK'})            
            smach.StateMachine.add('PARK', WaitOut('park', 180), transitions={'succeeded':'PARK_DEPTHCHANGE2', 'failed':'park_failed'})
            smach.StateMachine.add('PARK_DEPTHCHANGE2', GoToDepth(10,0.5), transitions={'succeeded':'park_complete'})
        smach.StateMachine.add('PARK_TASK', park, transitions={'park_complete':'LANE_PARK_TASK', 'park_failed':'SURFACE'})

        lane_park = smach.StateMachine(outcomes=['lane_complete', 'lane_failed'])
        with lane_park:
            smach.StateMachine.add('LANE_HOVER', HoverSearch('lane', 10, True, 2), transitions={'succeeded':'LANE_STORE', 'failed':'LANE_SEARCH'})            
            smach.StateMachine.add('LANE_SEARCH', LinearSearch('lane', 40, 1.5, 'sway', True, 2), transitions={'succeeded':'LANE_STORE', 'failed':'LANE_SEARCH2'})
            smach.StateMachine.add('LANE_SEARCH2', LinearSearch('lane', 40, -3, 'sway', True, 2), transitions={'succeeded':'LANE_STORE', 'failed':'lane_failed'})            
            smach.StateMachine.add('LANE_STORE', StoreGlobalCoord('mission_lane_park1'), transitions={'succeeded':'LANE_PARK'})
            smach.StateMachine.add('LANE_PARK', WaitOut('lane', 60), transitions={'succeeded':'LANE_HEADINGCHANGE', 'failed':'lane_failed'})
            smach.StateMachine.add('LANE_HEADINGCHANGE', GoToHeading(10), transitions={'succeeded':'lane_complete'})
        smach.StateMachine.add('LANE_PARK_TASK', lane_park, transitions={'lane_complete':'TOLL_TASK', 'lane_failed':'SURFACE'})  

        toll = smach.StateMachine(outcomes=['toll_complete', 'toll_failed'])
        with toll:
            smach.StateMachine.add('TOLL_DEPTHCHANGE', GoToDepth(3,2), transitions={'succeeded':'TOLL_SEARCH'})
            smach.StateMachine.add('TOLL_SEARCH', LinearSearch('tollbooth', 30, 4, 'fwd'), transitions={'succeeded':'TOLL_STORE', 'failed':'TOLL_SEARCH2'})
            smach.StateMachine.add('TOLL_SEARCH2', LinearSearch('tollbooth', 30, -2, 'sway'), transitions={'succeeded':'TOLL_STORE', 'failed':'TOLL_SEARCH3'})
            smach.StateMachine.add('TOLL_SEARCH3', LinearSearch('tollbooth', 60, 4, 'sway'), transitions={'succeeded':'TOLL_STORE', 'failed':'TOLL_SEARCH4'})
            smach.StateMachine.add('TOLL_SEARCH4', LinearSearch('tollbooth', 30, 2, 'fwd'), transitions={'succeeded':'TOLL_STORE', 'failed':'TOLL_SEARCH5'})
            smach.StateMachine.add('TOLL_SEARCH5', LinearSearch('tollbooth', 60, -4, 'sway'), transitions={'succeeded':'TOLL_STORE', 'failed':'TOLL_SEARCH6'})
            smach.StateMachine.add('TOLL_SEARCH6', LinearSearch('tollbooth', 60, 4, 'sway'), transitions={'succeeded':'TOLL_STORE', 'failed':'toll_failed'})
            smach.StateMachine.add('TOLL_STORE', StoreGlobalCoord('toll'), transitions={'succeeded':'TOLLBOOTH'})            
            smach.StateMachine.add('TOLLBOOTH', WaitOut('tollbooth', 180), transitions={'succeeded':'toll_complete', 'failed':'toll_failed'})
        smach.StateMachine.add('TOLL_TASK', toll, transitions={'toll_complete':'SPEED_TASK', 'toll_failed':'SPEED_TASK'})

        speed = smach.StateMachine(outcomes=['speed_complete', 'speed_failed'])
        with speed:
            smach.StateMachine.add('SPEED_DEPTHCHANGE', GoToDepth(10,0.5), transitions={'succeeded':'SPEED_SEARCH'})
#            smach.StateMachine.add('SPEED_SWAY', GoToDistance(90, -4, 'sway'), transitions={'succeeded':'speed_complete'})
            smach.StateMachine.add('SPEED_SEARCH', LinearSearch('speedtrap', 60, -5, 'sway'), transitions={'succeeded':'SPEED_STORE', 'failed':'SPEED_SEARCH2'})
            smach.StateMachine.add('SPEED_SEARCH2', LinearSearch('speedtrap', 30, -2, 'fwd'), transitions={'succeeded':'SPEED_STORE', 'failed':'SPEED_SEARCH3'})
            smach.StateMachine.add('SPEED_SEARCH3', LinearSearch('speedtrap', 60, 5, 'sway'), transitions={'succeeded':'SPEED_STORE', 'failed':'SPEED_SEARCH4'})
            smach.StateMachine.add('SPEED_SEARCH4', LinearSearch('speedtrap', 30, 2, 'fwd'), transitions={'succeeded':'SPEED_STORE', 'failed':'SPEED_SEARCH5'})
            smach.StateMachine.add('SPEED_SEARCH5', LinearSearch('speedtrap', 60, -5, 'sway'), transitions={'succeeded':'SPEED_STORE', 'failed':'speed_failed'})
#            smach.StateMachine.add('SPEED_TO_LANE3', NavMoveBase(1,60,place='lane3'), transitions={'succeeded':'SPEED_FWDSEARCH', 'failed':'speed_failed'})
            smach.StateMachine.add('SPEED_STORE', StoreGlobalCoord('mission_speed'), transitions={'succeeded':'SPEEDTRAP'})
            smach.StateMachine.add('SPEEDTRAP', WaitOut('speedtrap', 240), transitions={'succeeded':'speed_complete', 'failed':'speed_failed'})           
        smach.StateMachine.add('SPEED_TASK', speed, transitions={'speed_complete':'BACK_TO_LANE3', 'speed_failed':'BACK_TO_LANE3'})

        smach.StateMachine.add('BACK_TO_LANE3', NavMoveBase(3,70,place='mission_lane_park1'), transitions={'succeeded':'BACK_TO_LANE2', 'failed':'mission_failed'})        
        smach.StateMachine.add('BACK_TO_LANE2', NavMoveBase(3,70,place='mission_lane_traffic'), transitions={'succeeded':'BACK_TO_LANE1', 'failed':'mission_failed'})        
        smach.StateMachine.add('BACK_TO_LANE1', NavMoveBase(3,70,place='mission_lane_gate'), transitions={'succeeded':'HOME', 'failed':'mission_failed'})
        smach.StateMachine.add('HOME', NavMoveBase(1,60,0,0,0.5,26), transitions={'succeeded':'SURFACE', 'failed':'mission_failed'})
        smach.StateMachine.add('SURFACE', GoToDepth(3,0.12), transitions={'succeeded':'mission_complete'})
