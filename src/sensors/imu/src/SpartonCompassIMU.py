#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
#from geometry_msgs.msg import Pose2D
from bbauv_msgs.msg import imu_data

import serial, string, math, time, calendar

#import tf
from tf.transformations import *
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

gyroSampleRate = 107.95 #hard-coded gyroSampleRate

def wrapTo2PI(theta):
    #   Normalize an angle in radians to [0, 2*pi]
    return theta % (2.*math.pi)

def wrapToPI(theta):
    #   Normalize an angle in radians to [-pi, pi]
    return (wrapTo2PI(theta+math.pi) - math.pi)

def Spartonshutdownhook():
    global D_Compass
    global myStr1
    print "Sparton shutdown time!"
    D_Compass.write(myStr1) # stop data stream before close port
    D_Compass.flush() # flush data out
    rospy.loginfo('Closing Digital Compass Serial port')
    D_Compass.close() #Close D_Compass serial port
    # rospy.on_shutdown(Spartonshutdownhook)

if __name__ == '__main__':
    global D_Compass
    global myStr1
    rospy.init_node('SpartonDigitalCompassIMU')
    #Pos_pub = rospy.Publisher('AHRS8_HeadingTrue', Pose2D)
    Imu_pub_q = rospy.Publisher('AHRS8_data_q', Imu)
    Imu_pub_e = rospy.Publisher('AHRS8_data_e', imu_data)
    Imu_pub_temp = rospy.Publisher('AHRS8_Temp', Float32)    
    #SpartonPose2D=Pose2D()
    #SpartonPose2D.x=float(0.0)
    #SpartonPose2D.y=float(0.0)
    #Init D_Compass port
    D_Compassport = rospy.get_param('~port','/dev/ttyAHRS')
    D_Compassrate = rospy.get_param('~baud',115200)
    # printmodulus set to 1 is 100 Hz. 2 : 50Hz 
    D_Compassprintmodulus = rospy.get_param('~printmodulus', 40)
    #Digital compass heading offset in degree
    D_Compass_offset = rospy.get_param('~offset',0.)
    Imu_data = Imu()
    imu_data = imu_data()
    temp = Float32()
    
    Imu_data = Imu(header=rospy.Header(frame_id="AHRS8"))
    
    #TODO find a right way to convert imu acceleration/angularvel./orientation accuracy to covariance
    Imu_data.orientation_covariance = [1e-3, 0, 0, 
                                       0, 1e-3, 0, 
                                       0, 0, 1e-3]
    
    Imu_data.angular_velocity_covariance = [1e-3, 0, 0,
                                            0, 1e-3, 0, 
                                            0, 0, 1e-3]
    
    Imu_data.linear_acceleration_covariance = [1e-3, 0, 0, 
                                               0, 1e-3, 0, 
                                               0, 0, 1e-3]
    myStr1='\r\n\r\nprinttrigger 0 set drop\r\n'
    myStr2='printmask gyrop_trigger accelp_trigger or quat_trigger or yawt_trigger or time_trigger or temp_trigger or set drop\r\n'
        # set the number high to get lower update rate , the IMU data is 100Hz rate , the string is 130 byte with 10 bit/byte , the max sampling rate is 88Hz
        # printmodulus=2 might give us 50Hz update rate ( I have no idea ) ,set printmodulus=1 should give you the max speed. ( with auto skiping )
    myStr_printmodulus=('printmodulus %i set drop\r\n' % D_Compassprintmodulus  )
    myStr3='printtrigger printmask set drop\r\n'

    rospy.on_shutdown(Spartonshutdownhook)

    try:
        #talker()
        #ReadCompass()
        #Setup Compass serial por
        D_Compass = serial.Serial(port=D_Compassport, baudrate=D_Compassrate, timeout=.5)
        # Stop continus mode
        D_Compass.write(myStr1)
        D_Compass.flush() # flush data out
        time.sleep(0.5)
        # readout all data, if any
        rospy.loginfo("Send Stop Continus mode to Digital Compass Got bytes %i" % D_Compass.inWaiting() ) # should got OK here
        if (D_Compass.inWaiting() >0):
                #read out all datas, the response shuldbe OK
                data=D_Compass.read(D_Compass.inWaiting())
                print("Send to Digital Compass: %s Got: %s" % (myStr1 ,data)) # should got OK here

        else:
                #sned error no data in buffer error
                rospy.logerr('[1]Received No data from DigitalCompass. Shutdown!')
                rospy.signal_shutdown('Received No data from DigitalCompass')
        D_Compass.write(myStr2) # send printmask
        data = D_Compass.readline()
#        rospy.loginfo("Send to Digital Compass: %s" % myStr2 ) # should got OK here
#        rospy.loginfo("Send to Digital Compass Got: %s" % data ) # should got OK here
        if (len(data) >0):
                #read out all datas, the response shuldbe OK
                rospy.loginfo("Send to Digital Compass: %s" % myStr2 ) # should got OK here
                rospy.loginfo("Send to Digital Compass. Got: %s" % data ) # should got OK here
                D_Compass.write(myStr_printmodulus) # setup printmodule
                data = D_Compass.readline()
                rospy.loginfo("Send to Digital Compass: %s " % myStr_printmodulus) # should got OK here
                rospy.loginfo("Send to Digital Compass. Got: %s" % data) # should got OK here

                D_Compass.write(myStr3) # start the data streaming
                data = D_Compass.readline()
                rospy.loginfo("Send to Digital Compass: %s " % myStr3 ) # should got OK here
                rospy.loginfo("Send to Digital Compass. Got: %s" % data) # should got OK here
                rospy.loginfo('Digital Compass Setup Complete!')
        else:
                #sned error no data in buffer
                rospy.logerr('[2]Received No data from DigitalCompass. Shutdown!')
                rospy.signal_shutdown('Received No data from DigitalCompass')

        #data = D_Compass.readline()
        #Read in D_Compass
        #Testdata='P:,%i,ap,-6.34,-22.46,1011.71,gp,0.00,0.00,-0.00,yt,342.53,q,0.98,-0.01,0.01,-0.15\n'
        #i=0

        heading_array = []
        while not rospy.is_shutdown():
            #read D_Compass line  , The data example $HCHDT,295.5,T*2B
            #                                        [0]    [1] 
            #i+=1
            #D_Compass.write(Testdata % i) # send testdata and do loop-back in RS232 for debug
            data = D_Compass.readline()
            #rospy.loginfo("Received a sentence: %s" % data)

            #if not check_checksum(data):
            #    rospy.logerr("Received a sentence with an invalid checksum. Sentence was: %s" % data)
            #    continue

            #DatatimeNow = rospy.get_rostime()
            DataTimeSec=rospy.get_time()
            fields = data.split(',')
            #print fields[0]+fields[2]+fields[6]+fields[10]+fields[12] #P:apgpytq

            try:
                if len(fields) > 18:
                        if 'P:apgpytqT' == (fields[0]+fields[2]+fields[6]+fields[10]+fields[12]+fields[17]):

                                #      0  1 mSec 2  3Ax  4Ay     5Az     5  7Gx  8Gy  9G    10 11YawT 1213w  14x   15y  16z
                                #data='P:,878979,ap,-6.34,-22.46,1011.71,gp,0.00,0.00,-0.00,yt,342.53,q,0.98,-0.01,0.01,-0.15'

                                # notice the coordinate transformation
                                Ax=float(fields[3])/1000. # convert to g/s from mg/s
                                Ay=float(fields[4])/1000.
                                Az=float(fields[5])/1000.
                                Gx=float(fields[7]) * gyroSampleRate
                                Gy=float(fields[8]) * gyroSampleRate
                                Gz=float(fields[9]) * gyroSampleRate
                                w =float(fields[13])
                                x =float(fields[14])
                                y =float(fields[15])
                                z =float(fields[16])
                                T =float(fields[18])
                               
                                #imu_data.header.stamp = rospy.Time.now() # Should add an offset here
                                Imu_data.header.stamp = rospy.Time.from_sec(DataTimeSec-len(data)/11520.) # this is timestamp with a bit time offset 10bit per byte @115200bps 
                                # Euler message dataaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
                                # orientation
                                imu_data.orientation.x = math.atan2(2. * (w*x + y*z), 1-2. * (x*x + y*y))
                                imu_data.orientation.y = math.asin(2. * (w*y - z*x))
                                imu_data.orientation.z = (2.*math.pi + math.atan2(2.*(w*z + x*y), 1-2.*(y*y + z*z))) % (2.*math.pi)
                                
                                # angular velocity
                                imu_data.angular_velocity.x = Gx
                                imu_data.angular_velocity.y = Gy
                                imu_data.angular_velocity.z = Gz

                                # acceleration
                                imu_data.linear_acceleration.x = Ax
                                imu_data.linear_acceleration.y = Ay
                                imu_data.linear_acceleration.z = Az

				                # temperature
                                temp.data = T;

                                # Quaternion message dataaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
                                Imu_data.orientation = Quaternion()
                                # orientation
                                #ai = imu_data.orientation.x / 2.0
                                #aj = imu_data.orientation.y / 2.0
                                #ak = imu_data.orientation.z / 2.0
                                #ci = math.cos(ai)
                                #si = math.sin(ai)
                                #cj = math.cos(aj)
                                #sj = math.sin(aj)
                                #ck = math.cos(ak)
                                #sk = math.sin(ak)
                                #quaternion = numpy.empty((4, ), dtype=numpy.float64)
                                
                                Imu_data.orientation.x = x #si*cj*ck - ci*sj*sk
                                Imu_data.orientation.y = y #ci*sj*ck + si*cj*sk
                                Imu_data.orientation.z = z #ci*cj*sk - si*sj*ck
                                Imu_data.orientation.w = w #ci*cj*ck + si*sj*sk
                                
                                # angular velocity
                                Imu_data.angular_velocity.x = Gx
                                Imu_data.angular_velocity.y = Gy
                                Imu_data.angular_velocity.z = Gz
                                
                                # acceleration
                                Imu_data.linear_acceleration.x = Ax
                                Imu_data.linear_acceleration.y = Ay
                                Imu_data.linear_acceleration.z = Az

                                if len(heading_array) > 5:
                                    heading_array.reverse()
                                    heading_array.pop()
                                    heading_array.reverse()
                                heading_array.append(imu_data.orientation.z)
                                imu_data.orientation.z = sum(heading_array) / len(heading_array)
                                # Publish dataaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
                                Imu_pub_q.publish(Imu_data)
                                Imu_pub_e.publish(imu_data)
                                Imu_pub_temp.publish(temp)

                                #SpartonPose2D.y=1000./(float(fields[1])-SpartonPose2D.x) # put update rate here for debug the update rate
                                #SpartonPose2D.x=float(fields[1]) # put mSec tick here for debug the speed
                                #SpartonPose2D.theta = wrapToPI(math.radians(90.-float(fields[11])-D_Compass_offset))

                                #Pos_pub.publish(SpartonPose2D)


                        else:
                                rospy.logerr("[3]Received a sentence but not correct. Sentence was: %s" % data)
                else:
                        rospy.logerr("[4]Received a sentence but not correct. Sentence was: %s" % data)

            except ValueError as e:
                rospy.logwarn("Value error, likely due to missing fields in the data messages.Sentence was: %s %s" % (data, fields[0]+fields[2]+fields[6]+fields[10]+fields[12]+fields[17]))


            # no loop, delay, ROSspin() here, we try to read all the data asap
        D_Compass.write(myStr1) # stop data stream before close port
        D_Compass.flush() # flush data out

        rospy.loginfo('Closing Digital Compass Serial port')
        D_Compass.close() #Close D_Compass serial port
    except rospy.ROSInterruptException:
        pass
