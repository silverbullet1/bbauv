from v4_mission_planner import *

if __name__ == '__main__':
    
    sm_mission = smach.StateMachine(outcomes=['mission_complete','mission_failed'])

    with sm_mission:
        smach.StateMachine.add('COUNTDOWN', Countdown(0), transitions={'succeeded':'START'})
        smach.StateMachine.add('START',Start(5,0.6,120),
                                transitions={'succeeded':'LANE_GATE_TASK'})

        lane_gate = smach.StateMachine(outcomes=['lane_complete', 'lane_failed'])
        with lane_gate:

            zigzagSearch = Sequence(outcomes=['succeeded', 'failed'], connector_outcome= 'failed')
            with zigzagSearch:
                smach.StateMachine.add('SEARCH', LinearSearch('lane', 30, 2, 'fwd', False, 1))
                smach.StateMachine.add('SEARCH2', LinearSearch('lane', 30, -1, 'sway', False, 1))
                smach.StateMachine.add('SEARCH3', LinearSearch('lane', 30, 1, 'sway', False, 1))
            smach.StateMachine.add('ZIGZAGSEARCH', zigzagSearch, transitions={'succeeded':'STORE', 'failed':'lane_failed'})

#            smach.StateMachine.add('SEARCH', LinearSearch('lane', 30, 2, 'fwd', False, 1), transitions={'succeeded':'STORE', 'failed':'SEARCH2'})
#            smach.StateMachine.add('SEARCH2', LinearSearch('lane', 20, -1, 'sway', False, 1), transitions={'succeeded':'STORE', 'failed':'SEARCH3'})
#            smach.StateMachine.add('SEARCH3', LinearSearch('lane', 20, 1, 'sway', False, 1), transitions={'succeeded':'STORE', 'failed':'lane_failed'})
            smach.StateMachine.add('STORE', StoreGlobalCoord('mission_lane_gate'), transitions={'succeeded':'LANE_GATE'})
            smach.StateMachine.add('LANE_GATE', WaitOut('lane', 60), transitions={'succeeded':'HEADINGCHANGE', 'failed':'lane_failed'})
            smach.StateMachine.add('HEADINGCHANGE', GoToHeading(10), transitions={'succeeded':'lane_complete'})
        smach.StateMachine.add('LANE_GATE_TASK', lane_gate, transitions={'lane_complete':'ABOUTURN', 'lane_failed':'TURNTOTOLL'}) 

        smach.StateMachine.add('ABOUTURN', GoToHeading(10, 180, relative=True), transitions={'succeeded':'TRAFFIC_TASK'})
        smach.StateMachine.add('TURNTOTOLL', GoToHeading(10, 90), transitions={'succeeded':'TOLL_TASK'})

        trafficTaskTimeOut = 180
        traffic = smach.StateMachine(outcomes=['traffic_complete', 'traffic_failed'])
        with traffic:
            smach.StateMachine.add('DEPTHCHANGE', GoToDepth(15,1), transitions={'succeeded':'SEARCH'})
            smach.StateMachine.add('SEARCH', HoverSearch('traffic', 60), transitions={'succeeded':'STORE', 'failed':'traffic_failed'})
            smach.StateMachine.add('STORE', StoreGlobalCoord('mission_traffic'), transitions={'succeeded':'TRAFFIC'})
            smach.StateMachine.add('TRAFFIC', WaitOut('traffic', trafficTaskTimeOut), transitions={'succeeded':'DEPTHCHANGE2', 'failed':'traffic_failed'})
            smach.StateMachine.add('DEPTHCHANGE2', GoToDepth(15,0.6), transitions={'succeeded':'traffic_complete'})
        smach.StateMachine.add('TRAFFIC_TASK', traffic, transitions={'traffic_complete':'TURNTOTOLL', 'traffic_failed':'TURNTOTOLL'})

        toll = smach.StateMachine(outcomes=['toll_complete', 'toll_failed'])
        with toll:
            smach.StateMachine.add('GOFWD', GoToDistance(10, 2, 'fwd'), transitions={'succeeded':'DEPTHCHANGE'})
            smach.StateMachine.add('DEPTHCHANGE', GoToDepth(10,1.1), transitions={'succeeded':'SEARCH'})
            smach.StateMachine.add('SEARCH', LinearSearch('tollbooth', 30, 1, 'sway'), transitions={'succeeded':'STORE', 'failed':'SEARCH2'})
            smach.StateMachine.add('SEARCH2', LinearSearch('tollbooth', 30, -1, 'sway'), transitions={'succeeded':'STORE', 'failed':'toll_failed'})
            smach.StateMachine.add('STORE', StoreGlobalCoord('mission_toll'), transitions={'succeeded':'TOLLBOOTH'})            
            smach.StateMachine.add('TOLLBOOTH', WaitOut('tollbooth', 180), transitions={'succeeded':'toll_complete', 'failed':'toll_failed'})
        smach.StateMachine.add('TOLL_TASK', toll, transitions={'toll_complete':'GOFWD', 'toll_failed':'TOLLFAILNAV'})

        smach.StateMachine.add('TOLLFAILNAV', Nav(30,60,30,-2,2.5, 0.6,0.8, 100), transitions={'succeeded':'SPEED_TASK', 'failed':'mission_failed'})
        smach.StateMachine.add('GOFWD', GoToDistance(15,1,'fwd'), transitions={'succeeded':'SPEED_TASK'})

        speed = smach.StateMachine(outcomes=['speed_complete', 'speed_failed'])
        with speed:
            smach.StateMachine.add('DEPTHCHANGE', GoToDepth(10,0.6), transitions={'succeeded':'TURN'})
            smach.StateMachine.add('TURN', GoToHeading(15,90), transitions={'succeeded':'ZIGZAGSEARCH'})

            zigzagSearch = Sequence(outcomes=['succeeded', 'failed'], connector_outcome= 'failed')
            with zigzagSearch:
                smach.StateMachine.add('SEARCH', LinearSearch('speedtrap', 60, 3.5, 'sway'))
                smach.StateMachine.add('SEARCH2', LinearSearch('speedtrap', 30, 1, 'fwd'))
                smach.StateMachine.add('SEARCH3', LinearSearch('speedtrap', 60, -3.5, 'sway'))
            smach.StateMachine.add('ZIGZAGSEARCH', zigzagSearch, transitions={'succeeded':'STORE', 'failed':'speed_failed'})
              
            smach.StateMachine.add('STORE', StoreGlobalCoord('mission_speed'), transitions={'succeeded':'SPEEDTRAP'})
            smach.StateMachine.add('SPEEDTRAP', WaitOut('speedtrap', 150), transitions={'succeeded':'speed_complete', 'failed':'speed_failed'})           
        smach.StateMachine.add('SPEED_TASK', speed, transitions={'speed_complete':'PARK_TASK', 'speed_failed':'PARK_TASK'})

        park = smach.StateMachine(outcomes=['park_complete', 'park_failed'])
        with park:
            smach.StateMachine.add('DEPTHCHANGE', GoToDepth(10,1), transitions={'succeeded':'TURN'})
            smach.StateMachine.add('TURN', GoToHeading(15,270), transitions={'succeeded':'SEARCH'})
            smach.StateMachine.add('SEARCH', HoverSearch('park', 20), transitions={'succeeded':'STORE', 'failed':'SEARCH2'})
            smach.StateMachine.add('SEARCH2', LinearSearch('park', 60, -3, 'sway'), transitions={'succeeded':'STORE', 'failed':'park_failed'})
            smach.StateMachine.add('STORE', StoreGlobalCoord('mission_park1'), transitions={'succeeded':'PARK'})            
            smach.StateMachine.add('PARK', WaitOut('park', 90), transitions={'succeeded':'DEPTHCHANGE2', 'failed':'park_failed'})
            smach.StateMachine.add('DEPTHCHANGE2', GoToDepth(10,0.6), transitions={'succeeded':'park_complete'})
        smach.StateMachine.add('PARK_TASK', park, transitions={'park_complete':'GETOUT_OF_GATE', 'park_failed':'GETOUT_OF_GATE'})

        smach.StateMachine.add('GETOUT_OF_GATE', Nav(30,60,30,-2,2, 0.5,1,270),transitions={'succeeded':'PARK2_TASK', 'failed':'PARK2_TASK'})

        park2 = smach.StateMachine(outcomes=['park_complete', 'park_failed'])
        with park2:
            smach.StateMachine.add('DEPTHCHANGE', GoToDepth(10,1), transitions={'succeeded':'TURN'})
            smach.StateMachine.add('TURN', GoToHeading(10,270), transitions={'succeeded':'SEARCH'})
            smach.StateMachine.add('SEARCH', HoverSearch('park', 20), transitions={'succeeded':'STORE', 'failed':'SEARCH2'})
            smach.StateMachine.add('SEARCH2', LinearSearch('park', 60, -3, 'sway'), transitions={'succeeded':'STORE', 'failed':'park_failed'})
            smach.StateMachine.add('STORE', StoreGlobalCoord('mission_park1'), transitions={'succeeded':'PARK'})            
            smach.StateMachine.add('PARK', WaitOut('park', 90), transitions={'succeeded':'DEPTHCHANGE2', 'failed':'park_failed'})
            smach.StateMachine.add('DEPTHCHANGE2', GoToDepth(10,0.6), transitions={'succeeded':'park_complete'})
        smach.StateMachine.add('PARK2_TASK', park2, transitions={'park_complete':'GETOUT_OF_GATE2', 'park_failed':'GETOUT_OF_GATE2'})

        smach.StateMachine.add('GETOUT_OF_GATE2', Nav(30,60,30,-2,2, 0.5,1,270),transitions={'succeeded':'DRIVE_THRU_TASK', 'failed':'DRIVE_THRU_TASK'})

        drive_thru = smach.StateMachine(outcomes=['drive_complete', 'drive_failed'])
        with drive_thru:
            
            #Searching for Pinger
            smach.StateMachine.add('DEPTHCHANGE_PINGER', GoToDepth(10,0.7), transitions={'succeeded':'TURN'})
            smach.StateMachine.add('TURN', GoToHeading(15,270), transitions={'succeeded':'HOVER'})
            
            #Creep Search
            smach.StateMachine.add('HOVER', HoverSearch('acoustic', 60), transitions={'succeeded':'PINGER', 'failed':'GOFWD'})
            smach.StateMachine.add('GOFWD', GoToDistance(40, 4, 'fwd'), transitions={'succeeded':'HOVER2'})
            smach.StateMachine.add('HOVER2', HoverSearch('acoustic', 60), transitions={'succeeded':'PINGER', 'failed':'GOFWD2'})    
            smach.StateMachine.add('GOFWD2', GoToDistance(40, 3, 'fwd'), transitions={'succeeded':'HOVER3'})
            smach.StateMachine.add('HOVER3', HoverSearch('acoustic', 60), transitions={'succeeded':'PINGER', 'failed':'drive_failed'})
    
            smach.StateMachine.add('PINGER', WaitOut('acoustic', 210), transitions={'succeeded':'DEPTHCHANGE', 'failed':'DEPTHCHANGE'})    
            smach.StateMachine.add('PICKUP', WaitOut('drivethru', 60), transitions={'succeeded':'NAV2', 'failed':'drive_failed'})

            smach.StateMachine.add('DEPTHCHANGE', GoToDepth(10,0.6), transitions={'succeeded':'HOVER4'})   
            smach.StateMachine.add('HOVER4', HoverSearch('drivethru', 10), transitions={'succeeded':'PICKUP' , 'failed':'SEARCH_LEFT'})
            
            #Star Search Pattern
            smach.StateMachine.add('SEARCH_LEFT', LinearSearch('drivethru', 30, -1.5, 'sway'), transitions={'succeeded':'PICKUP', 'failed':'SEARCH_RIGHT'})
            smach.StateMachine.add('SEARCH_RIGHT', LinearSearch('drivethru', 30, 3, 'sway'), transitions={'succeeded':'PICKUP', 'failed':'SEARCH_LEFT2'})
            smach.StateMachine.add('SEARCH_LEFT2', LinearSearch('drivethru', 30, -1.5, 'sway'), transitions={'succeeded':'PICKUP', 'failed':'SEARCH_FRONT'})
            smach.StateMachine.add('SEARCH_FRONT', LinearSearch('drivethru', 30, 1.5, 'fwd'), transitions={'succeeded':'PICKUP', 'failed':'SEARCH_REAR'})
            smach.StateMachine.add('SEARCH_REAR', LinearSearch('drivethru', 30, -1.5, 'fwd'), transitions={'succeeded':'PICKUP', 'failed':'drive_failed'})

            smach.StateMachine.add('NAV2', Nav(30,60,30,-2,-2,0.5,0.5,70), transitions={'succeeded':'HOVER5', 'failed':'drive_failed'})
            smach.StateMachine.add('HOVER5', HoverSearch('acoustic', 70), transitions={'succeeded':'PINGER2', 'failed':'drive_failed'})
            smach.StateMachine.add('PINGER2', WaitOut('acoustic', 180), transitions={'succeeded':'HOVER6', 'failed':'drive_failed'})
            smach.StateMachine.add('HOVER6', HoverSearch('drivethru', 70), transitions={'succeeded':'DROPIT', 'failed':'drive_failed'})
            smach.StateMachine.add('DROPIT', WaitOut('drivethru', 60), transitions={'succeeded':'drive_complete', 'failed':'drive_failed'})
            
        smach.StateMachine.add('DRIVE_THRU_TASK', drive_thru, transitions={'drive_complete' : 'HOME' , 'drive_failed': 'HOME' })
        
        smach.StateMachine.add('HOME', Nav(30,60,30,-0.25,0.25,0.5,0.5,0), transitions={'succeeded':'SURFACE_VICTORIOUSLY', 'failed':'mission_failed'})
        smach.StateMachine.add('SURFACE_VICTORIOUSLY', GoToDepth(5, 0.27), transitions={'succeeded':'END'})
        #smach.StateMachine.add('VICTORY_SPIN', GoToHeading(60, 0, relative=True), transitions={'succeeded':'END'})
        smach.StateMachine.add('END', End(), transitions={'succeeded':'mission_complete'})
        