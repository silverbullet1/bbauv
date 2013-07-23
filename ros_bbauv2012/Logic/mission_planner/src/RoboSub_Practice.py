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
            sP = {'tN':'lane', 'fTo':10, 'sTo': 15,
                  'sqL': 1,
                  'isL':False , 'nL': '1'}
            squareSearch = Sequence(outcomes=['succeeded', 'failed'], connector_outcome= 'failed')
            with squareSearch:
                Sequence.add('FWD', LinearSearch(zP['tN'], zP['fTo'], zP['sqL']/float(2) , 'fwd', zP['isL'], zP['nL']))
                Sequence.add('RIGHT', LinearSearch(zP['tN'], zP['sTo'], zP['sqL']/float(2) , 'sway', zP['isL'], zP['nL']))
                Sequence.add('BACK', LinearSearch(zP['tN'], zP['fTo'], -1*zP['sqL'] , 'fwd', zP['isL'], zP['nL']))
                Sequence.add('LEFT', LinearSearch(zP['tN'], zP['sTo'], -1*zP['sqL'] , 'sway', zP['isL'], zP['nL']))
                Sequence.add('FWD2', LinearSearch(zP['tN'], zP['fTo'], zP['sqL'] , 'fwd', zP['isL'], zP['nL']))
                Sequence.add('RIGHT2', LinearSearch(zP['tN'], zP['sTo'], zP['sqL']/float(2) , 'sway', zP['isL'], zP['nL']))
            StateMachine.add('SQSEARCH', squareSearch, transitions={'succeeded':'TASK_EXECUTION', 'failed':'lane_failed'}) 
            
            task = {'tN':'lane', 'tOut':60}
            task_execution = Sequence(outcome=['succeeded', 'failed'], connector_outcome = 'succeeded')
            with task_execution:
                StateMachine.add('STORE_FOUND', StoreGlobalCoord('mission_traffic_found'))
                StateMachine.add(task['tN'], WaitOut(task['tN'], task['tOut']))
                StateMachine.add('STORE_DONE', StoreGlobalCoord('mission_traffic_done'))                               
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

            task = {'tN':'traffic', 'tOut':100}
            task_execution = Sequence(outcome=['succeeded', 'failed'], connector_outcome = 'succeeded')
            with task_execution:
                Sequence.add('STORE_FOUND', StoreGlobalCoord('mission_traffic_found'))
                Sequence.add(task['tN'], WaitOut(task['tN'], task['tOut']))
                Sequence.add('STORE_DONE', StoreGlobalCoord('mission_traffic_done'))                               
            StateMachine.add('TASK_EXECUTION', task_execution, transitions={'succeeded':'lane_complete', 'failed':'lane_failed'})
            
        smach.StateMachine.add('TRAFFIC', traffic, transitions={'traffic_complete':'LANE_TRAFFIC', 'traffic_failed':'SURFACE'})                    
###################################################################
        lane_traffic = StateMachine(outcome=['lane_failed', 'lane_failed'])
        with lane_traffic:
            StateMachine.add('DEPTHCHANGE', GoToDepth(5,0.6), transitions={'succeeded':'GOFWD'})
            StateMachine.add('GOFWD', GoToDistance(10, 1.5, 'fwd'), transitions={'succeeded':'GORIGHT'})
            StateMachine.add('GOLEFT', GoToDistance(10, 2, 'sway'), transitions={'succeeded':'HOVER'})
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

            task = {'tN':'lane', 'tOut':60}
            task_execution = Sequence(outcome=['succeeded', 'failed'], connector_outcome = 'succeeded')
            with task_execution:
                Sequence.add('STORE_FOUND', StoreGlobalCoord('mission_lane_traffic_found'))
                Sequence.add(task['tN'], WaitOut(task['tN'], task['tOut']))
                Sequence.add('STORE_DONE', StoreGlobalCoord('mission_lane_traffic_done'))                               
            StateMachine.add('TASK_EXECUTION', task_execution, transitions={'succeeded':'lane_complete', 'failed':'lane_failed'})

            smach.StateMachine.add('ZIGZAGSEARCH', zigzagSearch, transitions={'succeeded':'TASK_EXECUTION', 'failed':'traffic_failed'})                        
                                    
        StateMachine.add('LANE_TRAFFIC', lane_traffic, transitions = {'succeeded':'PARK_TASK', 'failed':''})        
###################################################################

        park = StateMachine(outcome=['succeeded', 'failed'])
        with park:
            StateMachine.add('DEPTHCHANGE', GoToDepth(5,0.6), transitions={'succeeded':'GOFWD'})
            StateMachine.add('GOFWD', GoToDistance(10, 1.5, 'fwd'), transitions={'succeeded':'HOVER'})
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
            
            task = {'tN':'lane', 'tOut':60}
            task_execution = Sequence(outcome=['succeeded', 'failed'], connector_outcome = 'succeeded')
            with task_execution:
                Sequence.add('STORE_FOUND', StoreGlobalCoord('mission_lane_traffic_found'))
                Sequence.add(task['tN'], WaitOut(task['tN'], task['tOut']))
                Sequence.add('STORE_DONE', StoreGlobalCoord('mission_lane_traffic_done'))                               
            StateMachine.add('TASK_EXECUTION', task_execution, transitions={'succeeded':'lane_complete', 'failed':'lane_failed'})
                        
        StateMachine.add('PARK_TASK', park, transitions={'succeeded':'', 'failed':''}])

###################################################################
        
        #Mission Ending
        StateMachine.add('SURFACE', GoToDepth(5, 0.27), transitions={'succeeded':'END'}) 
        StateMachine.add('END_SADLY', End(), transitions={'succeeded':'mission_failed'})
        StateMachine.add('END_VICTORIOUSLY', End(), transitions={'succeeded':'mission_complete'})
