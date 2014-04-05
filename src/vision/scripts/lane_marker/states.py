import roslib; roslib.load_manifest('vision')
import rospy

import smach, smach_ros

from com import Com

class Disengage(smach.State):
    def __init__(self):
        pass
    def execute(self, userdata):
        pass

def main():
    rospy.init_node('lane_marker_node')
    myCom = Com()

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'killed'])
    with sm:
        pass

    introServer = smach_ros.IntrospectionServer('mission_server', sm, '/MISSION/LANE_MARKER')
    introServer.start()

    sm.execute()
