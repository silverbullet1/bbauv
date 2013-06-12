#!/usr/bin/env python

import roslib; roslib.load_manifest('Openups')
import rospy
import pycanberra
#from bbauv_msgs.msg import battery_info
from bbauv_msgs.msg import openups

def destroyCanberra():
    canberra.destroy()  

def main():

    charge_status = {'oUPS1':'0','oUPS2':'0','oUPS3':'0','oUPS4':'0'}

    def callback(data):

        charge_status['oUPS1'] = data.battery1
        charge_status['oUPS2'] = data.battery2
        charge_status['oUPS3'] = data.battery3
        charge_status['oUPS4'] = data.battery4

    rospy.init_node('battery_notifier', anonymous=True)
    rospy.Subscriber("openups", openups, callback)

    while not rospy.is_shutdown():
        
        for key,value in charge_status.iteritems():
            if value <= 10:
                canberra = pycanberra.Canberra()
                canberra.easy_play_sync("suspend-error")
                canberra.destroy()  
                rospy.loginfo('Low Power')

        rospy.sleep(1)
        rospy.on_shutdown(destroyCanberra)

if __name__ == '__main__':
    main()
