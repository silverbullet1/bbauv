from v4_mission_planner import *

#import StateMachine, add fail state to STORE 

if __name__ == '__main__':

    sm_mission = StateMachine(outcomes=['mission_complete','mission_failed'])
    
    with sm_mission:
        StateMachine.add('COUNTDOWN', Countdown(0), transitions={'succeeded':'START'})
        StateMachine.add('START',Start(1,0.5,210), transitions={'succeeded':'NAV_TO_GATE'})
        
        StateMachine.add('NAV_TO_GATE', Nav(1,1,-9,-8.7,0.5,210), transitions={'succeeded':'LANE_GATE', 'failed':'SURFACE'})   
        
###################################################################        
        lane_gate = StateMachine(outcomes=['lane_complete', 'lane_failed'])
        with lane_gate:
        
            StateMachine.add('HOVER', HoverSearch('lane', 3, False, 1), transitions={'succeeded':'TASK_EXECUTION', 'failed':'SQSEARCH'})
            
            #Square Search
            sP = {'tN':'lane', 'fTo':20, 'sTo': 20,
                  'sqL': 1,
                  'isL':False , 'nL': '1'}
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
            task_execution = Sequence(outcome=['succeeded', 'failed'], connector_outcome = 'succeeded')
            with task_execution:
                StateMachine.add('STORE_FOUND', StoreGlobalCoord('mission_laneGate_found'))
                Sequence.add(task['tN'], WaitOut(task['tN'], task['tOut'], task['bL']))
                StateMachine.add('STORE_DONE', StoreGlobalCoord('mission_laneGate_done'))                               
            StateMachine.add('TASK_EXECUTION', task_execution, transitions={'succeeded':'lane_complete', 'failed':'lane_failed'})
                                        
        StateMachine.add('LANE_GATE', lane_gate, transitions={'lane_complete':'TRAFFIC', 'lane_failed':'SURFACE'})
        
###################################################################        
        traffic = StateMachine(outcomes=['traffic_complete', 'traffic_failed'])
        with traffic:
            StateMachine.add('DEPTHCHANGE', GoToDepth(5,2.5), transitions={'succeeded':'HOVER'})
            StateMachine.add('HOVER', HoverSearch('lane', 3, False, 1), transitions={'succeeded':'TASK_EXECUTION', 'failed':'ZIGZAGSEARCH'})

            #Zig Zag Search
            zP = {'tN':'traffic', 'fTo':20, 'sTo': 10, 
                  'fD':2, 'sD': 1, 
                  'isL':False , 'nL': '1'}
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

            task = {'tN':'traffic', 'tOut':60, 'bL':10}
            task_execution = Sequence(outcome=['succeeded', 'failed'], connector_outcome = 'succeeded')
            with task_execution:
                Sequence.add('STORE_FOUND', StoreGlobalCoord('mission_traffic_found'))
                Sequence.add(task['tN'], WaitOut(task['tN'], task['tOut'], task['bL']))
                Sequence.add('STORE_DONE', StoreGlobalCoord('mission_traffic_done'))                               
            StateMachine.add('TASK_EXECUTION', task_execution, transitions={'succeeded':'lane_complete', 'failed':'lane_failed'})
            
        smach.StateMachine.add('TRAFFIC', traffic, transitions={'traffic_complete':'LANE_TRAFFIC', 'traffic_failed':'SURFACE'})                    
###################################################################
        lane_traffic = StateMachine(outcome=['lane_complete', 'lane_failed'])
        with lane_traffic:
            StateMachine.add('DEPTHCHANGE', GoToDepth(5,0.6), transitions={'succeeded':'GOFWD'})
            StateMachine.add('GOFWD', GoToDistance(10, 1.5, 'fwd'), transitions={'succeeded':'GORIGHT'})
            StateMachine.add('GORIGHT', GoToDistance(10, 2, 'sway'), transitions={'succeeded':'HOVER'})
            StateMachine.add('HOVER', HoverSearch('lane', 5, False, 1), transitions={'succeeded':'TASK_EXECUTION', 'failed':'ZIGZAGSEARCH'})            
            
            #Zig Zag Search
            zP = {'tN':'lane', 'fTo':5, 'sTo': 30, 
                  'fD':0.5, 'sD': 4, 
                  'isL':False , 'nL': '1'}
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

            task = {'tN':'lane', 'tOut':60, 'bL':10}
            task_execution = Sequence(outcome=['succeeded', 'failed'], connector_outcome = 'succeeded')
            with task_execution:
                Sequence.add('STORE_FOUND', StoreGlobalCoord('mission_laneTraffic_found'))
                Sequence.add(task['tN'], WaitOut(task['tN'], task['tOut'], task['bL']))
                Sequence.add('STORE_DONE', StoreGlobalCoord('mission_laneTraffic_done'))                               
            StateMachine.add('TASK_EXECUTION', task_execution, transitions={'succeeded':'lane_complete', 'failed':'lane_failed'})

            smach.StateMachine.add('ZIGZAGSEARCH', zigzagSearch, transitions={'succeeded':'TASK_EXECUTION', 'failed':'traffic_failed'})                        
                                    
        StateMachine.add('LANE_TRAFFIC', lane_traffic, transitions = {'lane_complete':'PARK', 'lane_failed':'SURFACE'})    
                    
###################################################################

        park = StateMachine(outcome=['park_complete', 'park_failed'])
        with park:
            StateMachine.add('DEPTHCHANGE', GoToDepth(5,0.6), transitions={'succeeded':'GOFWD'})
            StateMachine.add('GOFWD', GoToDistance(10, 1.5, 'fwd'), transitions={'succeeded':'HOVER'})
            StateMachine.add('HOVER', HoverSearch('park', 5, False, 1), transitions={'succeeded':'TASK_EXECUTION', 'failed':'ZIGZAGSEARCH'})
            
            #Zig Zag Search
            zP = {'tN':'park', 'fTo':5, 'sTo': 30, 
                  'fD':0.5, 'sD': 4, 
                  'isL':False , 'nL': '1'}
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
            
            task = {'tN':'park', 'tOut':60, 'bL':10}
            task_execution = Sequence(outcome=['succeeded', 'failed'], connector_outcome = 'succeeded')
            with task_execution:
                Sequence.add('STORE_FOUND', StoreGlobalCoord('mission_park1_found'))
                Sequence.add(task['tN'], WaitOut(task['tN'], task['tOut'], task['bL']))
                Sequence.add('STORE_DONE', StoreGlobalCoord('mission_park1_done'))                               
            StateMachine.add('TASK_EXECUTION', task_execution, transitions={'succeeded':'lane_complete', 'failed':'park_failed'})
                        
        StateMachine.add('PARK', park, transitions={'park_complete':'LANE_PARK', 'park_failed':'NAVTO_LISTENING_POST'}])

###################################################################

        lane_park = StateMachine(outcomes=['lane_complete','lane_failed'])
        with lane_park:

            StateMachine.add('HOVER', HoverSearch('lane', 3, False, 2), transitions={'succeeded':'TASK_EXECUTION', 'failed':'LOOK_LEFT'})
            StateMachine.add('LOOK_LEFT', LinearSearch('lane', 30, -2, 'sway', False, 2), transitions={'succeeded':'TASK_EXECUTION', 'failed':'LOOK_RIGHT'})
            StateMachine.add('LOOK_RIGHT', LinearSearch('lane', 30, 4, 'sway', False, 2), transitions={'succeeded':'TASK_EXECUTION', 'failed':'lane_failed'})                       

            task = {'tN':'lane', 'tOut':60, 'bL':10}
            task_execution = Sequence(outcome=['succeeded', 'failed'], connector_outcome = 'succeeded')
            with task_execution:
                StateMachine.add('STORE_FOUND', StoreGlobalCoord('mission_traffic_found'))
                Sequence.add(task['tN'], WaitOut(task['tN'], task['tOut'], task['bL']))
                StateMachine.add('STORE_DONE', StoreGlobalCoord('mission_traffic_done'))                               
            StateMachine.add('TASK_EXECUTION', task_execution, transitions={'succeeded':'lane_complete', 'failed':'lane_failed'})
            
        StateMachine.add('LANE_PARK', lane_park, transitions={'lane_complete':'TOLLBOOTH', 'lane_failed':'NAVTO_LISTENING_POST'})

###################################################################
        
        tollbooth = StateMachine(outcomes=['toll_complete', 'toll_failed'])
        with tollbooth:

            StateMachine.add('DEPTHCHANGE', GoToDepth(5,2.5), transitions={'succeeded':'ZIGZAGSEARCH'})

            #Zig Zag Search
            zP = {'tN':'tollbooth', 'fTo':20, 'sTo': 40, 
                  'fD':2, 'sD': 5, 
                  'isL':False , 'nL': '1'}
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
            
            task = {'tN':'tollbooth', 'tOut':180, 'bL':10}
            task_execution = Sequence(outcome=['succeeded', 'failed'], connector_outcome = 'succeeded')
            with task_execution:
                StateMachine.add('STORE_FOUND', StoreGlobalCoord('mission_toll_found'))
                Sequence.add(task['tN'], WaitOut(task['tN'], task['tOut'], task['bL']))
                StateMachine.add('STORE_DONE', StoreGlobalCoord('mission_toll_done'))                             
            StateMachine.add('TASK_EXECUTION', task_execution, transitions={'succeeded':'toll_complete', 'failed':'toll_failed'})            
            
        StateMachine.add('TOLLBOOTH', tollbooth, transitions={'toll_complete':'SPEEDTRAP', 'toll_failed':'NAVTO_LISTENING_POST'})

###################################################################
        
        #EDIT LISTENING POST COORD HERE!!!!!!!!
        StateMachine.add('NAVTO_LISTENING_POST', Nav(30,60,10,x=30,y=30,0.6,0.6,0), transitions = {'succeeded':'DRIVE_THRU', 'failed':'DRIVE_THRU'})        

###################################################################
        
        #Mission Ending
        StateMachine.add('SURFACE', GoToDepth(5, 0.27), transitions={'succeeded':'END'}) 
        StateMachine.add('END_SADLY', End(), transitions={'succeeded':'mission_failed'})
        StateMachine.add('END_VICTORIOUSLY', End(), transitions={'succeeded':'mission_complete'})
