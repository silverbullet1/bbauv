from v4_mission_planner import *

if __name__ == '__main__':
    
    sm_mission = smach.StateMachine(outcomes=['mission_complete','mission_failed'])

    with sm_mission:
        smach.StateMachine.add('COUNTDOWN', Countdown(0), transitions={'succeeded':'START'})
        smach.StateMachine.add('START',Start(5,0.5,90),
                                transitions={'succeeded':'TOLL_TASK'})

        toll = smach.StateMachine(outcomes=['toll_complete', 'toll_failed'])
        with toll:
            smach.StateMachine.add('DEPTHCHANGE', GoToDepth(10,0.8), transitions={'succeeded':'SEARCH'})
            smach.StateMachine.add('SEARCH', LinearSearch('tollbooth', 30, 1, 'sway'), transitions={'succeeded':'STORE', 'failed':'SEARCH2'})
            smach.StateMachine.add('SEARCH2', LinearSearch('tollbooth', 30, -1, 'sway'), transitions={'succeeded':'STORE', 'failed':'toll_failed'})
            smach.StateMachine.add('STORE', StoreGlobalCoord('mission_toll'), transitions={'succeeded':'TOLLBOOTH'})            
            smach.StateMachine.add('TOLLBOOTH', WaitOut('tollbooth', 150), transitions={'succeeded':'toll_complete', 'failed':'toll_failed'})
        smach.StateMachine.add('TOLL_TASK', toll, transitions={'toll_complete':'SPEED_TASK', 'toll_failed':'TOLLFAILMOVEFWD'})

        smach.StateMachine.add('TOLLFAILMOVEFWD', GoToDistance(20,2,'fwd'), transitions={'succeeded':'SPEED_TASK'})

        speed = smach.StateMachine(outcomes=['speed_complete', 'speed_failed'])
        with speed:
            smach.StateMachine.add('DEPTHCHANGE', GoToDepth(10,0.5), transitions={'succeeded':'TURN'})
            smach.StateMachine.add('TURN', GoToHeading(15,90), transitions={'succeeded':'GOFWD'})
            smach.StateMachine.add('GOFWD', GoToDistance(15,0.5,'fwd'), transitions={'succeeded':'SEARCH'})
            smach.StateMachine.add('SEARCH', LinearSearch('speedtrap', 60, 3, 'sway'), transitions={'succeeded':'STORE', 'failed':'SEARCH2'})
            smach.StateMachine.add('SEARCH2', LinearSearch('speedtrap', 30, 1, 'fwd'), transitions={'succeeded':'STORE', 'failed':'SEARCH3'})
            smach.StateMachine.add('SEARCH3', LinearSearch('speedtrap', 60, -3, 'sway'), transitions={'succeeded':'STORE', 'failed':'speed_failed'})    
            smach.StateMachine.add('STORE', StoreGlobalCoord('mission_speed'), transitions={'succeeded':'SPEEDTRAP'})
            smach.StateMachine.add('SPEEDTRAP', WaitOut('speedtrap', 150), transitions={'succeeded':'speed_complete', 'failed':'speed_failed'})           
        smach.StateMachine.add('SPEED_TASK', speed, transitions={'speed_complete':'PARK_TASK', 'speed_failed':'PARK_TASK'})

        park = smach.StateMachine(outcomes=['park_complete', 'park_failed'])
        with park:
            smach.StateMachine.add('DEPTHCHANGE', GoToDepth(10,1), transitions={'succeeded':'TURN'})
            smach.StateMachine.add('TURN', GoToHeading(15,270), transitions={'succeeded':'SEARCH'})
            smach.StateMachine.add('SEARCH', HoverSearch('park', 60), transitions={'succeeded':'STORE', 'failed':'SEARCH2'})
            smach.StateMachine.add('SEARCH2', LinearSearch('park', 60, 2, 'sway'), transitions={'succeeded':'STORE', 'failed':'park_failed'})
            smach.StateMachine.add('STORE', StoreGlobalCoord('mission_park1'), transitions={'succeeded':'PARK'})            
            smach.StateMachine.add('PARK', WaitOut('park', 120), transitions={'succeeded':'DEPTHCHANGE2', 'failed':'park_failed'})
            smach.StateMachine.add('DEPTHCHANGE2', GoToDepth(10,0.5), transitions={'succeeded':'park_complete'})
        smach.StateMachine.add('PARK_TASK', park, transitions={'park_complete':'DRIVE_THRU_TASK', 'park_failed':'DRIVE_THRU_TASK'})

        drive_thru = smach.StateMachine(outcomes=['drive_complete', 'drive_failed'])
        with drive_thru:
            
            #Searching for Pinger
            smach.StateMachine.add('TURN', GoToHeading(15,20), transitions={'succeeded':'HOVER'})
            smach.StateMachine.add('HOVER', HoverSearch('acoustic', 30), transitions={'succeeded':'PINGER', 'failed':'GOFWD'})
            smach.StateMachine.add('GOFWD', GoToDistance(40, 1, 'fwd'), transitions={'succeeded':'HOVER2'})
            smach.StateMachine.add('HOVER2', HoverSearch('acoustic', 30), transitions={'succeeded':'PINGER', 'failed':'GOFWD2'})    
            smach.StateMachine.add('GOFWD2', GoToDistance(40, 1, 'fwd'), transitions={'succeeded':'HOVER3'})
            smach.StateMachine.add('HOVER3', HoverSearch('acoustic', 30), transitions={'succeeded':'PINGER', 'failed':'drive_failed'})
    
            smach.StateMachine.add('PINGER', WaitOutAndSearch('acoustic','drivethru', 180), transitions={'task_succeeded':'HOVER4', 'search_succeeded':'PICKUP','failed':'drive_failed'})
    
            smach.StateMachine.add('PICKUP', WaitOut('drivethru', 60), transitions={'succeeded':'drive_complete', 'failed':'drive_failed'})
    
            smach.StateMachine.add('HOVER4', HoverSearch('drivethru', 120), transitions={'succeeded':'PICKUP' , 'failed':'SEARCH_LEFT'})    
            smach.StateMachine.add('SEARCH_LEFT', LinearSearch('drivethru', 30, -1, 'sway'), transitions={'succeeded':'PICKUP', 'failed':'SEARCH_RIGHT'})
            smach.StateMachine.add('SEARCH_RIGHT', LinearSearch('drivethru', 30, 2, 'sway'), transitions={'succeeded':'PICKUP', 'failed':'drive_failed'})

        smach.StateMachine.add('DRIVE_THRU_TASK', drive_thru, transitions={'drive_complete' : 'HOME' , 'drive_failed': 'SURFACE_SADLY' })
        
        smach.StateMachine.add('HOME', Nav(30,60,30,0,0,0.5,0.5,0), transitions={'succeeded':'SURFACE_VICTORIOUSLY', 'failed':'SURFACE_SADLY'})
        smach.StateMachine.add('SURFACE_SADLY', GoToDepth(20, 0.27), transitions={'succeeded':'mission_failed'})
        smach.StateMachine.add('SURFACE_VICTORIOUSLY', GoToDepth(5, 0.27), transitions={'succeeded':'VICTORY_SPIN'})
        smach.StateMachine.add('VICTORY_SPIN', GoToHeading(60, 0, relative=True), transitions={'succeeded':'mission_complete'})
        
        
        
        
        
        
        
        
        
        
        
        
        