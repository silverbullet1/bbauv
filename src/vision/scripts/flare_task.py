'''
State Machine for the Flare task
'''

import roslib
import rospy
import actionlib
from rospy.timer import sleep

import smach
import smach_ros

from dynamic_reconfigure.server import Server
#from Vision.cfg import flareTaskConfig

import math
import os
import sys
import numpy as np

from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from flare_vision import Flare

#Global variables 
isStart = False 
isAbort = False
isEnd = False
isTestMode = False                  #If test mode then don't wait for mission call  
rosRate = None 
flare = None
VisionLoopCount = 0                 #Counter for number of times the image is being processed

mani_pub = None
movement_client = None
locomotionGoal = None

flare_params = {'flare_area':0, 'centering_x':0, 'centering_y':0}

#Starts off in disengage class
class Disengage(smach.State):
    client = None
    
    def __init__(self, flare):
        smach.State.__init__(self, outcomes=['start_complete', 'complete_outcome', 'aborted'], input_keys=['complete'])
        self.flare = flare
    
    def execute(self, userdata):
        if userdata.complete == True:
            isStart = False
            isEnd = False
            
            try: 
                if isTestMode == False:
                    resp = mission_srv_request(False, True, locomotionGoal)
                    rospy.loginfo("Flare task completed")
            except rospy.ServiceExeption, e:
                print "Service call failed: %s" % e
        
        while not rospy.is_shutdown():
            if isEnd:
                rospy.signal_shutdown("Shutting down Flare Node")
            if isStart:
                flare.register()
                rospy.info("Starting Flare")
                return 'start_complete'
            r.sleep()
        
        return 'aborted'
    
#Searches for the flare
class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes['search_complete', 'aborted', 'mission_abort'])
    
    def execute(self, userdata):
        #do stuff
        return 'search_complete'

#Bash towards the flare!
class Manuoevre(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes['manuoevre_complete', 'lost_flare', 'aborted', 'mission_abort'])
    def execute(self,userdata):
        #do stuff
        return 'manuoevre_complete'

'''
Main python thread
'''
    
def handle_srv(req):
    global isStart
    global isAbort
    global locomotionGoal
    global flare
    
    rospy.loginfo("Flare service handled")
    
    if req.start_request:
        rospy.loginfo("Flare is Start")
        isStart = True
        isAbort = False 
        #locomotionGoal = req.start_ctrl
    if req.abort_reqest:
        rospy.loginfo("Flare abort received")
        isAbort = True
        isStart = False
        flare.unregister()
    
    #To fill accordingly
    return mission_to_visionResponse(isStart, isAbort)
    
#Param config callback
def flareCallback(conig, level):
    for param in flare.yellow_params:
        flare.yellow_params[param] = config['yellow_' + param]
    isTestMode = config["testmode"]
    return config

#Utility function for normalising heading 
def normHeading(heading):
    if heading > 360:
        return heading - 360
    elif heading < 0:
        return heading + 360
    else:
        return heading 

if __name__ == '__main__':
    rospy.init_node("Flare", anonymous=False)
    isTestMode = rospy.get_param("~testmode", False)
    rosRate = rospy.Rate(20)
    flare = Flare()
    rospy.loginfo("Flare loaded!")
    
    #Link to motion
    movement_client = actionlib.SimpleActionClient("LocomotionServer", bbauv_msgs.msg.ControllerAction)
    movement_client.wait_for_server()
    mani_pub = rospy.Publisher("/manipulators", manipulator)
    
    #Services (TODO)
    #srv = Server(flareTaskConfig, flareCallback)
    vision_srv = rospy.Service("Flare_srv", mission_to_vision, handle_srv)
    rospy.loginfo("Flare srv initialised!")
    
    #Not testing, then wait for mission's call
    if not isTestMode: 
        rospy.loginfo("Waiting for mission service")
        rospy.wait_for_service("mission_srv")
        mission_srv_request = rospy.ServiceProxy('mission_srv', vision_to_mission, headers={'id':'3'})
        rospy.loginfo("Connected to mission srv!")
    
    #Create state machine container 
    sm = smach.StateMachine(outcomes=['complete_flare', 'aborted'])
    
    #Disengage, Search, Manuoevre
    with sm:
        smach.StateMachine.add("DISENGAGE", Disengage(flare_task),
                               transitions={'start_complete': SEARCH, 
                                            'complete_outcome': 'complete_flare', 
                                            'aborted': 'aborted'})
        
        smach.StateMachine.add("SEARCH", Search(flare_task),
                               transitions={'search_complete': MANUOEVRE, 'aborted': 'aborted', 
                                            'mission_abort': DISENGAGED})
    
        smach.StateMachine.add("MANUOEVRE", Manuoevre(flare_task),
                               transitions = {'manuoevre_complete': DISENGAGED,
                                              'lost_flare': SEARCH,
                                              'aborted': 'aborted',
                                              'mission_abort': DISENGAGED})
    
    outcomes = sm.execute()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down flare")
    pass
    