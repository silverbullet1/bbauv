#!/usr/bin/env python
import rospy, roslib
from nav_msgs.msg import Odometry
from bbauv_msgs.msg import imu_data
from math import sin, cos
roslib.load_manifest('dvl')

vnorth = veast = 0
vnorth_old = veast_old = 0

north = east = 0;
dt = 0.20;

cnorth = ceast = 0
cvno = cveo = 0
cvn = cve =0

yaw = 0

def DVLCallback(d):
    global vnorth, veast, vnorth_old, veast_old, north, east, dt
    global yaw, cnorth, ceast, cvno, cveo, cvn, cve
    vnorth_old = vnorth
    veast_old = veast

    vnorth = d.twist.twist.linear.x
    veast = d.twist.twist.linear.y

    #north += (vnorth + oldvnorth) * delta / 2.0;
    north += (vnorth + vnorth_old) * dt / 2.0
    east  += (veast + veast_old) * dt / 2.0

    #e_north = veast * cos(yaw) + vnorth * sin(yaw);
    #e_east = veast * sin(yaw) + vnorth * sin(yaw);


    cvno = cvn
    cveo = cve

    cvn = veast * cos(yaw + 1.57) + vnorth * sin(yaw)
    cve = veast * sin(yaw + 1.57) + vnorth * cos(yaw)

    #rospy.loginfo("%s %s", str(veast), str(cvn))

    cnorth += (cvn + cvno) * dt / 2.0;
    ceast += (cve + cveo) * dt / 2.0;

    rospy.loginfo("%s %s %s %s", str(north), str(east), str(cnorth), str(ceast))




def AHRSSub(d):
    global yaw
    yaw = d.orientation.z
    #rospy.loginfo(d.orientation.z)

if __name__ == "__main__":
    rospy.init_node("DVLTest", anonymous=True)

    DVL = rospy.Subscriber("/WH_DVL_data", Odometry, DVLCallback)
    AHRS = rospy.Subscriber("/AHRS8_data_e", imu_data, AHRSSub)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down.")

