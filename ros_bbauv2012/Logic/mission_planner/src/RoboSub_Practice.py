from v4_mission_planner import *

#import StateMachine, add fail state to STORE 

if __name__ == '__main__':

    sm_mission = StateMachine(outcomes=['mission_complete','mission_failed'])
    
    with sm_mission:
        StateMachine.add('COUNTDOWN', Countdown(0), transitions={'succeeded':'START'})
        
        #Competition Side
        StateMachine.add('START',Start(5,0.5,40), transitions={'succeeded':'TURN_TO_GATE'})
        StateMachine.add('TURN_TO_GATE', GoToHeading(5, 295), transitions={'succeeded':'GO_TO_GATE'}) #practice side is 295
        StateMachine.add('GO_TO_GATE', GoToDistance(70, 13, 'fwd'), transitions={'succeeded':'LANE_GATE'})
        
###################################################################        
        lane_gate = StateMachine(outcomes=['lane_complete', 'lane_failed'])
        with lane_gate:
        
            StateMachine.add('HOVER', HoverSearch('lane', 3, False, 2), transitions={'succeeded':'TASK_EXECUTION', 'failed':'LOOK_LEFT'})
            StateMachine.add('LOOK_LEFT', LinearSearch('lane', 20, -2, 'sway', False, 2), transitions={'succeeded':'TASK_EXECUTION', 'failed':'LOOK_RIGHT'})
            StateMachine.add('LOOK_RIGHT', LinearSearch('lane', 30, 4, 'sway', False, 2), transitions={'succeeded':'TASK_EXECUTION', 'failed':'SQSEARCH'})
                        
            #Square Search
            sP = {'tN':'lane', 'fTo':30, 'sTo': 40,
                  'sqL': 2,
                  'isL': False , 'nL': 1}
            squareSearch = Sequence(outcomes=['succeeded', 'failed'], connector_outcome= 'failed')
            with squareSearch:
                Sequence.add('FWD', LinearSearch(sP['tN'], sP['fTo'], sP['sqL']/float(2) , 'fwd', sP['isL'], sP['nL']))
                Sequence.add('RIGHT', LinearSearch(sP['tN'], sP['sTo'], sP['sqL']/float(2) , 'sway', sP['isL'], sP['nL']))
                Sequence.add('BACK', LinearSearch(sP['tN'], sP['fTo'], -1*sP['sqL'] , 'fwd', sP['isL'], sP['nL']))
                Sequence.add('LEFT', LinearSearch(sP['tN'], sP['sTo'], -1*sP['sqL'] , 'sway', sP['isL'], sP['nL']))
                Sequence.add('FWD2', LinearSearch(sP['tN'], sP['fTo'], sP['sqL'] , 'fwd', sP['isL'], sP['nL']))
                Sequence.add('RIGHT2', LinearSearch(sP['tN'], sP['sTo'], sP['sqL']/float(2) , 'sway', sP['isL'], sP['nL']))
            smach.StateMachine.add('SQSEARCH', squareSearch, transitions={'succeeded':'TASK_EXECUTION', 'failed':'lane_failed'})
            
            task = {'tN':'lane', 'tOut':60, 'bL':5}
            task_execution = Sequence(outcomes=['succeeded', 'failed'], connector_outcome = 'succeeded')
            with task_execution:
                Sequence.add('STORE_FOUND', StoreGlobalCoord('mission_laneGate_found'))
                Sequence.add(task['tN'], WaitOut(task['tN'], task['tOut'], task['bL']))
                Sequence.add('STORE_DONE', StoreGlobalCoord('mission_laneGate_done'))                               
            StateMachine.add('TASK_EXECUTION', task_execution, transitions={'succeeded':'HEADINGCHANGE', 'failed':'lane_failed'})

            StateMachine.add('HEADINGCHANGE', GoToHeading(10), transitions={'succeeded':'lane_complete'})
                                        
        StateMachine.add('LANE_GATE', lane_gate, transitions={'lane_complete':'TRAFFIC', 'lane_failed':'SURFACE_SAD'})
        
###################################################################        
        traffic = StateMachine(outcomes=['traffic_complete', 'traffic_failed'])
        with traffic:
            StateMachine.add('DEPTHCHANGE', GoToDepth(5,2.5), transitions={'succeeded':'HOVER'})
            StateMachine.add('GOFWD', GoToDistance(10,2,'fwd'), transitions={'succeeded':'HOVER'})
            StateMachine.add('HOVER', HoverSearch('lane', 5, False, 1), transitions={'succeeded':'TASK_EXECUTION', 'failed':'ZIGZAGSEARCH'})

            #Zig Zag Search
            zP = {'tN':'traffic', 'fTo':15, 'sTo': 15, 
                  'fD':2, 'sD': 1, 
                  'isL':False , 'nL': 1}
            zigzagSearch = Sequence(outcomes=['succeeded', 'failed'], connector_outcome= 'failed')
            with zigzagSearch:
                #Initial
                Sequence.add('LEFT_START', LinearSearch(zP['tN'], zP['sTo'], -1*zP['sD']/float(2) , 'sway', zP['isL'], zP['nL']))
                Sequence.add('RIGHT1', LinearSearch(zP['tN'], zP['sTo'], zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('FWD1', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                Sequence.add('LEFT2', LinearSearch(zP['tN'], zP['sTo'], -1*zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('FWD2', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                Sequence.add('RIGHT2', LinearSearch(zP['tN'], zP['sTo'], zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('FWD3', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                Sequence.add('LEFT3', LinearSearch(zP['tN'], zP['sTo'], -1*zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('FWD4', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                #Final
                Sequence.add('RIGHT_FINAL', LinearSearch(zP['tN'], zP['sTo'], zP['sD']/float(2) , 'sway', zP['isL'], zP['nL']))

            smach.StateMachine.add('ZIGZAGSEARCH', zigzagSearch, transitions={'succeeded':'TASK_EXECUTION', 'failed':'traffic_failed'})            

            task = {'tN':'traffic', 'tOut':120, 'bL':10}
            task_execution = Sequence(outcomes=['succeeded', 'failed'], connector_outcome = 'succeeded')
            with task_execution:
                Sequence.add('STORE_FOUND', StoreGlobalCoord('mission_traffic_found'))
                Sequence.add(task['tN'], WaitOut(task['tN'], task['tOut'], task['bL']))
                Sequence.add('STORE_DONE', StoreGlobalCoord('mission_traffic_done'))                               
            StateMachine.add('TASK_EXECUTION', task_execution, transitions={'succeeded':'traffic_complete', 'failed':'traffic_failed'})
            
        smach.StateMachine.add('TRAFFIC', traffic, transitions={'traffic_complete':'LANE_TRAFFIC', 'traffic_failed':'SURFACE_SAD'})                    
###################################################################
        lane_traffic = StateMachine(outcomes=['lane_complete', 'lane_failed'])
        with lane_traffic:
            StateMachine.add('DEPTHCHANGE', GoToDepth(5,0.6), transitions={'succeeded':'GOFWD'})
            StateMachine.add('GOFWD', GoToDistance(10, 1.5, 'fwd'), transitions={'succeeded':'GOLEFT'})
            StateMachine.add('GOLEFT', GoToDistance(20, 2, 'sway'), transitions={'succeeded':'HOVER'})
            StateMachine.add('HOVER', HoverSearch('lane', 3, False, 1), transitions={'succeeded':'TASK_EXECUTION', 'failed':'ZIGZAGSEARCH'})            
            
            #Zig Zag Search
            zP = {'tN':'lane', 'fTo':10, 'sTo': 45, 
                  'fD':0.5, 'sD': 4, 
                  'isL':False , 'nL': 1}
            zigzagSearch = Sequence(outcomes=['succeeded', 'failed'], connector_outcome= 'failed')
            with zigzagSearch:
                #Initial
                Sequence.add('LEFT_START', LinearSearch(zP['tN'], zP['sTo'], -1*zP['sD']/float(2) , 'sway', zP['isL'], zP['nL']))
                Sequence.add('RIGHT1', LinearSearch(zP['tN'], zP['sTo'], zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('FWD1', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                Sequence.add('LEFT2', LinearSearch(zP['tN'], zP['sTo'], -1*zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('FWD2', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                Sequence.add('RIGHT2', LinearSearch(zP['tN'], zP['sTo'], zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('FWD3', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                Sequence.add('LEFT3', LinearSearch(zP['tN'], zP['sTo'], -1*zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('FWD4', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                #Final
                Sequence.add('RIGHT_FINAL', LinearSearch(zP['tN'], zP['sTo'], zP['sD']/float(2) , 'sway', zP['isL'], zP['nL']))
            smach.StateMachine.add('ZIGZAGSEARCH', zigzagSearch, transitions={'succeeded':'TASK_EXECUTION', 'failed':'lane_failed'})

            task = {'tN':'lane', 'tOut':60, 'bL':5}
            task_execution = Sequence(outcomes=['succeeded', 'failed'], connector_outcome = 'succeeded')
            with task_execution:
                Sequence.add('STORE_FOUND', StoreGlobalCoord('mission_laneTraffic_found'))
                Sequence.add(task['tN'], WaitOut(task['tN'], task['tOut'], task['bL']))
                Sequence.add('STORE_DONE', StoreGlobalCoord('mission_laneTraffic_done'))                               
            StateMachine.add('TASK_EXECUTION', task_execution, transitions={'succeeded':'HEADINGCHANGE', 'failed':'lane_failed'})

            StateMachine.add('HEADINGCHANGE', GoToHeading(10), transitions={'succeeded':'lane_complete'})    
                                            
        StateMachine.add('LANE_TRAFFIC', lane_traffic, transitions = {'lane_complete':'PARK', 'lane_failed':'SURFACE_SAD'})    
                    
###################################################################

        park = StateMachine(outcomes=['park_complete', 'park_failed'])
        with park:
            StateMachine.add('DEPTHCHANGE', GoToDepth(5,2.5), transitions={'succeeded':'GOFWD'})
            StateMachine.add('GOFWD', GoToDistance(20, 3, 'fwd'), transitions={'succeeded':'HOVER'})
            StateMachine.add('HOVER', HoverSearch('park', 5, False, 1), transitions={'succeeded':'TASK_EXECUTION', 'failed':'ZIGZAGSEARCH'})
            
            #Zig Zag Search
            zP = {'tN':'park', 'fTo':10, 'sTo': 45,
                  'fD':0.5, 'sD': 4,
                  'isL':False , 'nL': 1}
            zigzagSearch = Sequence(outcomes=['succeeded', 'failed'], connector_outcome= 'failed')
            with zigzagSearch:
                #Initial
                Sequence.add('LEFT_START', LinearSearch(zP['tN'], zP['sTo'], -1*zP['sD']/float(2) , 'sway', zP['isL'], zP['nL']))
                Sequence.add('RIGHT1', LinearSearch(zP['tN'], zP['sTo'], zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('FWD1', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                Sequence.add('LEFT2', LinearSearch(zP['tN'], zP['sTo'], -1*zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('FWD2', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                Sequence.add('RIGHT2', LinearSearch(zP['tN'], zP['sTo'], zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('FWD3', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                Sequence.add('LEFT3', LinearSearch(zP['tN'], zP['sTo'], -1*zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('FWD4', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                #Final
                Sequence.add('RIGHT_FINAL', LinearSearch(zP['tN'], zP['sTo'], zP['sD']/float(2) , 'sway', zP['isL'], zP['nL']))
            smach.StateMachine.add('ZIGZAGSEARCH', zigzagSearch, transitions={'succeeded':'TASK_EXECUTION', 'failed':'park_failed'})
            
            task = {'tN':'park', 'tOut':100 'bL':10}
            task_execution = Sequence(outcomes=['succeeded', 'failed'], connector_outcome = 'succeeded')
            with task_execution:
                Sequence.add('STORE_FOUND', StoreGlobalCoord('mission_park1_found'))
                Sequence.add(task['tN'], WaitOut(task['tN'], task['tOut'], task['bL']))
                Sequence.add('STORE_DONE', StoreGlobalCoord('mission_park1_done'))                               
            StateMachine.add('TASK_EXECUTION', task_execution, transitions={'succeeded':'park_complete', 'failed':'park_failed'})
                        
        StateMachine.add('PARK', park, transitions={'park_complete':'LANE_PARK', 'park_failed':'NAVTO_LISTENING_POST'})

###################################################################

        lane_park = StateMachine(outcomes=['lane_complete','lane_failed'])
        with lane_park:

            StateMachine.add('HOVER', HoverSearch('lane', 5, False, 2), transitions={'succeeded':'TASK_EXECUTION', 'failed':'LOOK_LEFT'})
            StateMachine.add('LOOK_LEFT', LinearSearch('lane', 25, -2, 'sway', False, 2), transitions={'succeeded':'TASK_EXECUTION', 'failed':'LOOK_RIGHT'})
            StateMachine.add('LOOK_RIGHT', LinearSearch('lane', 45, 4, 'sway', False, 2), transitions={'succeeded':'TASK_EXECUTION', 'failed':'lane_failed'})                       

            task = {'tN':'lane', 'tOut':60, 'bL':5}
            task_execution = Sequence(outcomes=['succeeded', 'failed'], connector_outcome = 'succeeded')
            with task_execution:
                StateMachine.add('STORE_FOUND', StoreGlobalCoord('mission_lanePark_found'))
                Sequence.add(task['tN'], WaitOut(task['tN'], task['tOut'], task['bL']))
                StateMachine.add('STORE_DONE', StoreGlobalCoord('mission_lanePark_done'))                               
            StateMachine.add('TASK_EXECUTION', task_execution, transitions={'succeeded':'HEADINGCHANGE', 'failed':'lane_failed'})

            StateMachine.add('HEADINGCHANGE', GoToHeading(10), transitions={'succeeded':'lane_complete'})
            
        StateMachine.add('LANE_PARK', lane_park, transitions={'lane_complete':'TOLLBOOTH', 'lane_failed':'NAVTO_LISTENING_POST'})

###################################################################
        
        tollbooth = StateMachine(outcomes=['toll_complete', 'toll_failed'])
        with tollbooth:

            StateMachine.add('DEPTHCHANGE', GoToDepth(10,2.5), transitions={'succeeded':'GOFWD'})
            StateMachine.add('GOFWD', GoToDistance(20,2,'fwd'), transitions={'succeeded':'ZIGZAGSEARCH'})

            #Zig Zag Search
            zP = {'tN':'tollbooth', 'fTo':20, 'sTo': 60, 
                  'fD':2, 'sD': 7, 
                  'isL':False , 'nL': 1}
            zigzagSearch = Sequence(outcomes=['succeeded', 'failed'], connector_outcome= 'failed')
            with zigzagSearch:
                #Initial
                Sequence.add('LEFT_START', LinearSearch(zP['tN'], zP['sTo'], -1*zP['sD']/float(2) , 'sway', zP['isL'], zP['nL']))
                Sequence.add('RIGHT1', LinearSearch(zP['tN'], zP['sTo'], zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('FWD1', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                Sequence.add('LEFT2', LinearSearch(zP['tN'], zP['sTo'], -1*zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('FWD2', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                Sequence.add('RIGHT2', LinearSearch(zP['tN'], zP['sTo'], zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('FWD3', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                Sequence.add('LEFT3', LinearSearch(zP['tN'], zP['sTo'], -1*zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('FWD4', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                #Final
                Sequence.add('RIGHT_FINAL', LinearSearch(zP['tN'], zP['sTo'], zP['sD']/float(2) , 'sway', zP['isL'], zP['nL']))
            smach.StateMachine.add('ZIGZAGSEARCH', zigzagSearch, transitions={'succeeded':'TASK_EXECUTION', 'failed':'toll_failed'})
            
            task = {'tN':'tollbooth', 'tOut':180, 'bL':10}
            task_execution = Sequence(outcomes=['succeeded', 'failed'], connector_outcome = 'succeeded')
            with task_execution:
                Sequence.add('STORE_FOUND', StoreGlobalCoord('mission_toll_found'))
                Sequence.add(task['tN'], WaitOut(task['tN'], task['tOut'], task['bL']))
                Sequence.add('STORE_DONE', StoreGlobalCoord('mission_toll_done'))                             
            StateMachine.add('TASK_EXECUTION', task_execution, transitions={'succeeded':'toll_complete', 'failed':'toll_failed'})            
            
        StateMachine.add('TOLLBOOTH', tollbooth, transitions={'toll_complete':'SPEEDTRAP', 'toll_failed':'NAVTO_LISTENING_POST'})

###################################################################

        speed = StateMachine(outcomes=['speed_complete', 'speed_failed'])
        with speed:
            StateMachine.add('DEPTHCHANGE', GoToDepth(5,0.6), transitions={'succeeded':'ZIGZAGSEARCH'})

            #Zig Zag Search
            zP = {'tN':'speedtrap', 'fTo':20, 'sTo': 60, 
                  'fD':-1, 'sD': 7, 
                  'isL':False , 'nL': 1}
            zigzagSearch = Sequence(outcomes=['succeeded', 'failed'], connector_outcome= 'failed')
            with zigzagSearch:
                #Initial
                Sequence.add('LEFT_START', LinearSearch(zP['tN'], zP['sTo'], -1*zP['sD']/float(2) , 'sway', zP['isL'], zP['nL']))
                Sequence.add('RIGHT1', LinearSearch(zP['tN'], zP['sTo'], zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('FWD1', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                Sequence.add('LEFT2', LinearSearch(zP['tN'], zP['sTo'], -1*zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('FWD2', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                Sequence.add('RIGHT2', LinearSearch(zP['tN'], zP['sTo'], zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('FWD3', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                Sequence.add('LEFT3', LinearSearch(zP['tN'], zP['sTo'], -1*zP['sD'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('FWD4', LinearSearch(zP['tN'], zP['fTo'], zP['fD'] , 'fwd', zP['isL'], zP['nL']))
                #Final
                Sequence.add('RIGHT_FINAL', LinearSearch(zP['tN'], zP['sTo'], zP['sD']/float(2) , 'sway', zP['isL'], zP['nL']))
            smach.StateMachine.add('ZIGZAGSEARCH', zigzagSearch, transitions={'succeeded':'TASK_EXECUTION', 'failed':'speed_failed'})  
            
            task = {'tN':'speedtrap', 'tOut':180, 'bL':5}
            task_execution = Sequence(outcomes=['succeeded', 'failed'], connector_outcome = 'succeeded')
            with task_execution:
                Sequence.add('STORE_FOUND', StoreGlobalCoord('mission_speed_found'))
                Sequence.add(task['tN'], WaitOut(task['tN'], task['tOut'], task['bL']))
                Sequence.add('STORE_DONE', StoreGlobalCoord('mission_speed_done'))                             
            StateMachine.add('TASK_EXECUTION', task_execution, transitions={'succeeded':'speed_complete', 'failed':'speed_failed'})            
                      
        StateMachine.add('SPEEDTRAP', speed, transitions={'speed_complete':'DRIVE_THRU', 'speed_failed':'NAVTO_LISTENING_POST'})            
        
###################################################################        
        #EDIT LISTENING POST COORD HERE!!!!!!!!
        StateMachine.add('NAVTO_LISTENING_POST', Nav(30,60,30,30,10,0.6,0.6,5), transitions = {'succeeded':'DRIVE_THRU', 'failed':'DRIVE_THRU'})        

###################################################################

        drive = StateMachine(outcomes=['drive_complete', 'drive_failed'])
        with drive:
            StateMachine.add('HOVER', HoverSearch('acoustic', 60), transitions={'succeeded':'PINGER', 'failed':'GOFWD'})
            StateMachine.add('GOFWD', GoToDistance(30, 3, 'fwd'), transitions={'succeeded':'HOVER2'})
            StateMachine.add('HOVER2', HoverSearch('acoustic', 60), transitions={'succeeded':'PINGER', 'failed':'GOFWD2'})
            StateMachine.add('GOFWD2', GoToDistance(30, 3, 'fwd'), transitions={'succeeded':'HOVER3'})
            StateMachine.add('HOVER3', HoverSearch('acoustic', 60), transitions={'succeeded':'PINGER', 'failed':'GOFWD3'})
            StateMachine.add('GOFWD3', GoToDistance(30, 3, 'fwd'), transitions={'succeeded':'HOVER4'})
            StateMachine.add('HOVER4', HoverSearch('acoustic', 60), transitions={'succeeded':'PINGER', 'failed':'drive_failed'})
                             
            StateMachine.add('PINGER', WaitOut('acoustic', 240), transitions={'succeeded':'HOVER5', 'failed':'drive_failed'})
            
            StateMachine.add('HOVER5', HoverSearch('drivethru', 3), transitions={'succeeded':'PICKUP', 'failed':'SEARCH_LEFT'})    
            StateMachine.add('SEARCH_LEFT', LinearSearch('drivethru', 20, -1.5, 'sway'), transitions={'succeeded':'PICKUP', 'failed':'SEARCH_RIGHT'})
            StateMachine.add('SEARCH_RIGHT', LinearSearch('drivethru', 40, 3, 'sway'), transitions={'succeeded':'PICKUP', 'failed':'SEARCH_LEFT2'})
            StateMachine.add('SEARCH_LEFT2', LinearSearch('drivethru', 20, -1.5, 'sway'), transitions={'succeeded':'PICKUP', 'failed':'SEARCH_FRONT'})
            StateMachine.add('SEARCH_FRONT', LinearSearch('drivethru', 20, 1.5, 'fwd'), transitions={'succeeded':'PICKUP', 'failed':'SEARCH_REAR'})
            StateMachine.add('SEARCH_REAR', LinearSearch('drivethru', 20, -1.5, 'fwd'), transitions={'succeeded':'PICKUP', 'failed':'drive_failed'})            

            StateMachine.add('PICKUP', WaitOut('drivethru', 100), transitions={'succeeded':'SURFACE', 'failed':'drive_failed'})
            StateMachine.add('SURFACE', GoToDepth(5, 0.2, surface = True), transitions={'succeeded':'DIVE_AGAIN'})
            StateMachine.add('DIVE_AGAIN', GoToDepth(5, 0.6), transitions={'succeeded':'HOVER6'})
            StateMachine.add('HOVER6', HoverSearch('acoustic', 5), transitions={'succeeded':'PINGER2', 'failed':'drive_failed'})
            StateMachine.add('PINGER2', WaitOut('acoustic', 120), transitions={'succeeded':'SHORTHOVER', 'failed':'drive_failed'})

            StateMachine.add('SHORTHOVER', HoverSearch('drivethru', 0.5), transitions={'succeeded':'DROPIT', 'failed':'drive_failed'})    
            StateMachine.add('DROPIT', WaitOut('drivethru', 5), transitions={'succeeded':'drive_complete', 'failed':'drive_failed'})

        StateMachine.add('DRIVE_THRU', drive, transitions = {'drive_complete':'SURFACE_HAPPY', 'drive_failed':'SURFACE_SAD'})

###################################################################        
        #Mission Ending
        StateMachine.add('SURFACE_SAD', GoToDepth(5, 0.20), transitions={'succeeded':'END_SAD'})
        StateMachine.add('SURFACE_HAPPY', GoToDepth(5, -1), transitions={'succeeded':'END_HAPPY'})        
        StateMachine.add('END_SAD', End(), transitions={'succeeded':'mission_failed'})
        StateMachine.add('END_HAPPY', End(), transitions={'succeeded':'mission_complete'})
