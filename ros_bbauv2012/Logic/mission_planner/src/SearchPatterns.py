from v4_mission_planner import *

if __name__ == '__main__':
    
    #Zig Zag Search
    zP = {'tN':'lane', 'fTo':10, 'sTo': 20, 
          'fD':1, 'sD': 1 , 
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

    smach.StateMachine.add('ZIGZAGSEARCH', zigzagSearch, transitions={'succeeded':'', 'failed':''})

    #Square Search
    sP = {'tN':'lane', 'fTo':10, 'sTo': 20,
          'sqL': 2,
          'isL':False , 'nL': '1'}
    squareSearch = Sequence(outcomes=['succeeded', 'failed'], connector_outcome= 'failed')
    with squareSearch:
        Sequence.add('FWD', LinearSearch(sP['tN'], sP['fTo'], sP['sqL']/float(2) , 'fwd', sP['isL'], sP['nL']))
        Sequence.add('RIGHT', LinearSearch(sP['tN'], sP['sTo'], sP['sqL']/float(2) , 'sway', sP['isL'], sP['nL']))
        Sequence.add('BACK', LinearSearch(sP['tN'], sP['fTo'], -1*sP['sqL'] , 'fwd', sP['isL'], sP['nL']))
        Sequence.add('LEFT', LinearSearch(sP['tN'], sP['sTo'], -1*sP['sqL'] , 'sway', sP['isL'], sP['nL']))
        Sequence.add('FWD2', LinearSearch(sP['tN'], sP['fTo'], sP['sqL'] , 'fwd', sP['isL'], sP['nL']))
        Sequence.add('RIGHT2', LinearSearch(sP['tN'], sP['sTo'], sP['sqL']/float(2) , 'sway', sP['isL'], sP['nL']))
    smach.StateMachine.add('SQSEARCH', squareSearch, transitions={'succeeded':'', 'failed':''})
