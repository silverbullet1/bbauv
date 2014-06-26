#/usr/bin/env python

'''
Communication b/w ROS class and submodules
'''

import roslib; roslib.load_manifest('vision')
import rospy
from front_commons.frontComms import FrontComms
from vision import TorpedoVision

from bbauv_msgs.msg import controller, manipulator, depth
from bbauv_msgs.srv import mission_to_visionResponse, \
        mission_to_vision, vision_to_mission
        
from dynamic_reconfigure.server import Server as DynServer
from utils.config import torpedoConfig as Config

from sensor_msgs.msg import Image
from utils.utils import Utils

class Comms(FrontComms):
    
    isTesting = False
    isKilled = False
    isAborted = True
    isStart = False
    
    # Circle booleans
    foundCircles = False 
    foundSomething = False 
    foundCount = 0  # If more than a constant, then respond lost
    
    # Shooting parameters
    numShoot = 0    # Only given 2 shoots 
    centroidToShoot = None
    torpedoOffset = 0.07

    # Board parameters
    boardCentroid = (-1, -1)
    boardArea = 0
    boardDeltaX = 0
    boardDeltaY = 0

    lockedCentroid = (-1, -1)
    isCenteringState = False
    
    # Movement parameters
    radius = 0
    deltaX = 0
    deltaY = 0

    sonarDist = 10.0
    sonarBearing = 0.0

    state = None
    centerDiff = 0
    
    def __init__(self):
        FrontComms.__init__(self, TorpedoVision(comms=self))
        self.defaultDepth = 2.20
        self.depthFromMission = self.defaultDepth
        self.depthSub = rospy.Subscriber("/depth", depth, self.depthCallback)

        #self.dynServer = DynServer(Config, self.reconfigure)

        # Initialise mission planner 
        if not self.isAlone:
            self.comServer = rospy.Service("/torpedo/mission_to_vision", 
                                           mission_to_vision,
                                           self.handle_srv)
            rospy.loginfo("Waiting for mission planner")
            self.toMission = rospy.ServiceProxy("/torpedo/vision_to_mission",
                                                vision_to_mission)
            self.toMission.wait_for_service()   #Indefinitely waiting for timeout
        
    # Handle mission services
    def handle_srv(self, req):
        global isStart
        global isAborted
        global locomotionGoal
        
        rospy.loginfo("Torpedo Service handled")
        
        if req.start_request:
            rospy.loginfo("Torpedo starting")
            self.isStart = True
            self.isAborted = False
            self.canPublish = True

            self.defaultDepth = req.start_ctrl.depth_setpoint
            self.inputHeading = req.start_ctrl.heading_setpoint
            self.curHeading = self.inputHeading
            self.depthFromMission = self.defaultDepth
            self.sonarBearing = self.inputHeading

            rospy.loginfo("Received depth: {}".format(self.defaultDepth))
            rospy.loginfo("Received heading: {}".format(self.inputHeading))

            self.registerMission()
            
            return mission_to_visionResponse(start_response=True,
                                             abort_response=False,
                                             data=controller(heading_setpoint=
                                                             self.curHeading))
        
        elif req.abort_request:
            rospy.loginfo("Torpedo abort received")
            self.isAborted=True
            self.isStart = False
            self.canPublish = False 

            self.unregisterMission()
            self.sendMovement(forward=0.0, sidemove=0.0)
            rospy.loginfo("Aborted complete")
            
            return mission_to_visionResponse(start_response=False,
                                             abort_response=True,
                                             data=controller(heading_setpoint=
                                                             self.curHeading))
            

    def depthCallback(self, data):
        self.depth = data.depth

    def shootTopTorpedo(self):
        maniPub = rospy.Publisher("/manipulators", manipulator)
        maniPub.publish(0 | 1)
        rospy.loginfo("Firing top torpedo")
        rospy.sleep(rospy.Duration(0.3)) 

    def shootBotTorpedo(self):
        maniPub = rospy.Publisher("/manipulators", manipulator)
        maniPub.publish(0 | 2)
        rospy.loginfo("Firing bottom torpedo")
        rospy.sleep(rospy.Duration(0.3))       
    
    def reconfigure(self, config, level):
        rospy.loginfo("Received dynamic reconfigure request")
        self.params = {'loThreshold': (config.loH, config.loS, config.loV),
                       'hiThreshold': (config.hiH, config.hiS, config.hiV),
                       'sonarOffset': config.sonarOffset,
                       'torpedoOffset': config.torpedoOffset, 
                       'minContourArea': config.minContourArea }
        self.visionFilter.updateParams()
        return config
    
    def registerSonar(self):
        rospy.loginfo("SONAR SONAR")
        self.sonarSub = rospy.Subscriber("/sonar_image_jin", Image, self.sonarImageCallback)
        self.sonarPub = rospy.Publisher("/sonar_pub", Image)
        rospy.sleep(rospy.Duration(0.5))
    
    def sonarImageCallback(self, rosImg):
        outImg = self.visionFilter.gotSonarFrame(Utils.rosimg2cv(rosImg))
        if self.canPublish and outImg is not None:
            try:
                self.sonarPub.publish(Utils.cv2rosimg(outImg))
            except Exception, e:
                pass
                
        rospy.sleep(rospy.Duration(0.3))
        
    def unregisterSonar(self):
        self.sonarSub.unregister()
    
def main():
    pass