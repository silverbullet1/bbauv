
from bbauv_msgs.msg import *
from bbauv_msgs.srv import * 
from nav_msgs.msg import Odometry

        self.compass_sub = rospy.Subscriber("/euler", compass_data, self.compassHandler)
        self.pos_sub = rospy.Subscriber("WH_DVL_data", Odometry, self.posHandler)
        self.depth_sub = rospy.Subscriber("/depth", depth, self.depthHandler)
        #Calling ActionServer 
        self.actionClient = actionlib.SimpleActionClient('LocomotionServer', ControllerAction)
        if not isTest:
            try:
                self.actionClient.wait_for_server(timeout=rospy.Duration(30))
                rospy.loginfo("Waiting for action server")
            except:
                rospy.logerr("Timeout waiting for action server")

        self.controllerServer = rospy.ServiceProxy("/set_controller_srv", set_controller)
        
        #Calling ControllerServer 
        try:
            self.controllerServer.wait_for_service(timeout = 50)
        except:
            rospy.loginfo("Timeout waiting for controller server")

        self.controllerServer(forward=True, sidemove=True, heading=True, depth=True, pitch=True, roll=False, topside=False, navigation=False)

    #Handlers
    def compassHandler(self, data):
        self.heading = data.yaw

    def depthHandler(self, data):
        self.depth = data.depth

    def posHandler(self, data):
        self.pos['x'] = data.pose.pose.position.x
        self.pos['y'] = data.pose.pose.position.y
    
