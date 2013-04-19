#!/usr/bin/env python

import roslib; roslib.load_manifest('Vision')
import rospy

import cv2
from cv2 import cv as cv
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest
import numpy as np
import sys

# State Machine
import smach
from smach_ros import IntrospectionServer

class Recover():
    def __init__(self):
        print "Recovering"

class Align():
    def __init__(self):
        print "Aligning"

class SlowAhead():
    def __init__(self):
        print "Aye Sir"

class Standby():
    def __init__(self):
        print "standby"
        
class Initialization:
    def __init__(self):
        self.node_name = "Parking"
        rospy.init_node(self.node_name)
        rospy.loginfo("Starting Node " + str(self.node_name))
        print "init'ing'"

class Parking_Proc:
    def __init__(self):                
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback)    
        self.bridge = CvBridge()
        
        self.hmin = rospy.get_param("~hmin", 70)
        self.hmax = rospy.get_param("~hmax", 80)
        self.smin = rospy.get_param("~smin", 80)
        self.smax = rospy.get_param("~smax", 140)
        self.vmin = rospy.get_param("~vmin", 150)
        self.vmax = rospy.get_param("~vmax", 255)
        self.hsv = {'hmin': self.hmin,'hmax':self.hmax,'smin':self.smin,'smax':self.smax,'vmin':self.vmin,'vmax':self.vmax,}        

        self.a_maxval = rospy.get_param("~a_maxval",255)        
        self.a_method = rospy.get_param("~a_method",1)        
        self.a_threshtype = rospy.get_param("~a_threshtype",1)
        self.a_block_size = rospy.get_param("~a_blocksize",15)        
        self.a_constant = rospy.get_param("~a_constant", 5)
        self.threshold = rospy.get_param("~threshold", 0)   
        self.debug_mode = rospy.get_param("~debug_mode", True)       
        
        print "image processing "
        
    def image_callback(self, data):
        frame = self.convert_image(data)
        processed_image = self.process_image(frame)

        
    def convert_image(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv(ros_image, "bgr8")       
            return np.array(cv_image, dtype=np.uint8)
        except CvBridgeError, e:
            print e
            
    def process_image(self, frame):
        
        frame2 = frame3 = frame
        
        horizontal_green = self.hsv_thresholding(frame2, hmin=self.hmin, hmax=self.hmax, smin=self.smin, smax=self.smax, vmin=self.vmin, vmax=self.vmax)

#        adapt = self.adaptive_thresholding(frame3)
        
        cam = self.cam_shift(frame3)

#        if self.debug_mode:
        cv2.imshow("HSV Thresholded", horizontal_green)
#        cv2.imshow("Adaptive Thresholded", adapt)
                            
        cv.WaitKey(1)
        return frame

    def cam_shift(self, frame):
        # First blue the image
        blur = cv2.blur(frame, (5, 5))
        
        # Convert from RGB to HSV spave
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        
        # Create a mask using the current saturation and value parameters
        mask = cv2.inRange(hsv, np.array((38., self.smin, self.vmin)), np.array((75., 250., self.vmax)))
        
        x0, y0, w, h = [0,0,320,240]
        x1 = x0 + w
        y1 = y0 + h
        self.track_window = (x0, y0, x1, y1)
        hsv_roi = hsv[y0:y1, x0:x1]
        mask_roi = mask[y0:y1, x0:x1]
        self.hist = cv2.calcHist( [hsv_roi], [0], mask_roi, [16], [0, 180] )
        cv2.normalize(self.hist, self.hist, 0, 255, cv2.NORM_MINMAX);
        self.hist = self.hist.reshape(-1)
        self.show_hist()
        
        # If we have a histogram, tracking it with CamShift
        if self.hist is not None:
            # Compute the backprojection from the histogram
            backproject = cv2.calcBackProject([hsv], [0], self.hist, [0, 180], 1)
            
            # Mask the backprojection with the mask created earlier
            backproject &= mask

            # Threshold the backprojection
            ret, backproject = cv2.threshold(backproject, self.threshold, 255, cv.CV_THRESH_TOZERO)

            x, y, w, h = self.track_window
            if self.track_window is None or w <= 0 or h <=0:
                self.track_window = 0, 0, self.frame_width - 1, self.frame_height - 1
            
            # Set the criteria for the CamShift algorithm
            term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )
            
            # Run the CamShift algorithm
            self.track_box, self.track_window = cv2.CamShift(backproject, self.track_window, term_crit)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2,2))
        backproject = cv2.morphologyEx(backproject, cv2.MORPH_CLOSE, kernel,iterations=20)
        
        if self.debug_mode:
            # Display the resulting backprojection
            cv.NamedWindow("Backproject", 0)
            cv.MoveWindow("Backproject", 0, 240)
            cv2.imshow("Backproject", backproject)

        return backproject
        
    def show_hist(self):
        bin_count = self.hist.shape[0]
        bin_w = 24
        img = np.zeros((256, bin_count*bin_w, 3), np.uint8)
        for i in xrange(bin_count):
            h = int(self.hist[i])
            cv2.rectangle(img, (i*bin_w+2, 255), ((i+1)*bin_w-2, 255-h), (int(180.0*i/bin_count), 255, 255), -1)
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        cv2.imshow('Histogram', img)
        

    def hue_histogram_as_image(self, hist):
            """ Returns a nice representation of a hue histogram """
            histimg_hsv = cv.CreateImage((320, 200), 8, 3)
            
            mybins = cv.CloneMatND(hist.bins)
            cv.Log(mybins, mybins)
            (_, hi, _, _) = cv.MinMaxLoc(mybins)
            cv.ConvertScale(mybins, mybins, 255. / hi)
    
            w,h = cv.GetSize(histimg_hsv)
            hdims = cv.GetDims(mybins)[0]
            for x in range(w):
                xh = (180 * x) / (w - 1)  # hue sweeps from 0-180 across the image
                val = int(mybins[int(hdims * x / w)] * h / 255)
                cv2.rectangle(histimg_hsv, (x, 0), (x, h-val), (xh,255,64), -1)
                cv2.rectangle(histimg_hsv, (x, h-val), (x, h), (xh,255,255), -1)
    
            histimg = cv2.cvtColor(histimg_hsv, cv.CV_HSV2BGR)
            
            return histimg
	
    def adaptive_thresholding(self, input_frame, maxval=255, method=1, threshtype=1, block_size=15, constant=5):

        blur = cv2.GaussianBlur(input_frame, (3,3), 2)
        gray = cv2.cvtColor(input_frame, cv2.COLOR_BGR2GRAY)
        adaptive_threshed = cv2.adaptiveThreshold(gray, maxval, method, threshtype, block_size, constant)

        return adaptive_threshed
		
    def hsv_thresholding(self, input_frame, hmin=0, hmax=180, smin=0, smax=255, vmin=0, vmax=255):
        
        #Assume frame is BGR format
        hsv_img = cv2.cvtColor(input_frame,cv2.COLOR_BGR2HSV)
             
        #initialize matrix for HSV min and max values
        COLOR_MIN = np.array([hmin, smin, vmin],np.uint8)
        COLOR_MAX = np.array([hmax, smax, vmax],np.uint8)
        
        #Execute Thresholding
        hsv_threshed = cv2.inRange(hsv_img, COLOR_MIN, COLOR_MAX)
        
        return hsv_threshed

def main(args):
    try:
        Initialization()
        Parking_Proc()
        Standby()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()   

if __name__ == '__main__':
    main(sys.argv)

