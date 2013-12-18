#!/usr/bin/env python

import roslib; roslib.load_manifest('Openups')
import rospy
import pycanberra
from bbauv_msgs.msg import hull_status

def destroyCanberra():
    canberra.destroy() 

def main():

    water_status = {'waterSensor1':'0','waterSensor2':'0','waterSensor3':'0'}

    def callback(data):

        water_status['waterSensorA'] = data.WaterDetA
        water_status['waterSensorB'] = data.WaterDetB
        water_status['waterSensorC'] = data.WaterDetC
    
    rospy.init_node('leak_notifier', anonymous=True)
    rospy.Subscriber("/hull_status", hull_status, callback)

    while not rospy.is_shutdown():
        
        for key,value in water_status.iteritems():
            if value == True:
                canberra = pycanberra.Canberra()
                canberra.easy_play_sync("alarm-clock-elapsed") #for more sounds, see here: http://0pointer.de/public/sound-naming-spec.html#guidelines
                canberra.destroy()
                rospy.loginfo("Hey dude, you've got water in your TUBE man!")  
                
        rospy.sleep(1)
        rospy.on_shutdown(destroyCanberra)

if __name__ == '__main__':
    main()
