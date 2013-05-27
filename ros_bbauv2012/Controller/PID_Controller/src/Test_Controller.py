#! /usr/bin/env python

import roslib; roslib.load_manifest('PID_Controller')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import PID_Controller.msg

def doneCB(state,result):
    print "I'm done!"
    print result.forward_final;
    print result.heading_final;
    print result.sidemove_final;

def feedbackCB(feedback):
    print "feedback",feedback.forward_error;
    
def controller_client():
    print "Client!"
    # Waits until the action server has started up and started
    # listening for goals.
    
    # Waits for the server to finish performing the action.
    #client.wait_for_result()
if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('Test_Controller_Py')
        client = actionlib.SimpleActionClient('LocomotionServer', PID_Controller.msg.ControllerAction)
        client.wait_for_server()
    
         # Creates a goal to send to the action server.
        goal = PID_Controller.msg.ControllerGoal(forward_setpoint=1,heading_setpoint=0,depth_setpoint=0.5,sidemove_setpoint=0.3)

        # Sends the goal to the action sserver.
        client.send_goal(goal,doneCB,None,feedbackCB)
    
        #print "Result:",result.final_input
        while True:
            print "client looping!"
        rospy.spin()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"