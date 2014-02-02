## Flare vision class

from bbauv_msgs.msg import compass_data
from bbauv_msgs.msg import controller
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

import rospy
import math
import cv2 as cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

class Flare:
    debug = True
    red_params = {'lowerH': 0, 'lowerS': 0, 'lowerV':100, 'higherH': 77, 'higherS':195, 'higherV':251 }
    red_hist = None
    
    isCenteringState = False
    isAlignState = True
    
    bridge = None
    image_speed = None
    
    yaw = 0
    centroidx = 0
    centroidy = 0
    centroidx_list = None
    centroidy_list = None
    angleList = None
    max_area = 0
    
    #Necessary published methods 
    image_pub = None
    image_sub = None
    yaw_sub = None
        
    '''
    Utility Methods 
    '''
    
    #Convert ROS image to Numpy matrix for cv2 functions 
    def rosimg2cv(self, ros_image):
        frame = self.bridge.imgmsg_to_cv(ros_image, ros_image.encoding)
        return np.array(frame, dtype = np.uint8)
    
    '''
    Bucket Node vision methods
    '''
    def __init__(self, debug_state):
        self.debug = debug_state
        self.image_speed = rospy.get_param('~image', '/bot_cam/camera/image_rect_color')
        #TODO: Add histogram modes for debug
        self.bridge = CvBridge()
        rospy.loginfo("Flare ready")
            
    def register(self):
        self.image_pub = rospy.Publisher("/bot_camera/filter" , Image)
        self.image_sub = rospy.Subscriber(self.image_speed, Image, self.processImage)
        self.yaw_sub = rospy.Subsriber('/euler', compass_data, self.yaw_callback)
        rospy.loginfo("Topics registered")
        
    def unregister(self):
        self.image_pub.unregister()
        self.image_sub.unregister()
        self.yaw_sub.unregister()
        rospy.loginfo("Topics unregistered")
    
    def yaw_callback(self, msg):
        self.yaw = msg.yaw
    
    #Perform red thresholding
    def findTheBucket(self, image):
        #Convert ROS to CV image 
        try:
            cv_image = self.rosimg2cv(image)
        except CvBridgeError, e:
            print e
        out = cv_image.copy()                                   #Copy of image for display later
        cv_image = cv2.GaussianBlur(cv_image, ksize=(5, 5), sigmaX=0)
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)   #Convert to HSV image
        hsv_image = np.array(hsv_image, dtype=np.uint8)         #Convert to numpy array

        #Perform yellow thresholding
        lowerBound = np.array([self.red_params['lowerH'], self.red_params['lowerS'], self.red_params['lowerV']],np.uint8)
        higherBound = np.array([self.red_params['higherH'], self.red_params['higherS'], self.red_params['higherV']],np.uint8)
        contourImg = cv2.inRange(hsv_image, lowerBound, higherBound)
        
        #Noise removal
        erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, (13,13))
        contourImg = cv2.erode(contourImg, erodeEl)
        contourImg = cv2.dilate(contourImg, dilateEl)
      
        #Find centroids
        pImg = contourImg.copy()
        contours, hierachy = cv2.findContours(pImg, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        rectList = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.areaThresh:
                # Find center with moments
                mu = cv2.moments(contour, False)
                mu_area = mu['m00']
                centroidx = mu['m10']/mu_area
                centroidy = mu['m01']/mu_area
                
                self.rectData['area'] = area
                self.rectData['centroids'] = (centroidx, centroidy)
                self.rectData['rect'] = cv2.minAreaRect(contour)
    
                points = np.array(cv2.cv.BoxPoints(self.rectData['rect']))
              
                #Find angle
                edge1 = points[1] - points[0]
                edge2 = points[2] - points[1]
                if cv2.norm(edge1) > cv2.norm(edge2):
                    edge1[1] = edge2[1] if edge2[1] is not 0 else 0.01
                    self.rectData['angle'] = math.degrees(math.atan(edge1[0]/edge1[1]))
                else:
                    edge1[1] = edge2[1] if edge2[1] is not 0 else 0.01
                    self.rectData['angle'] = math.degrees(math.atan(edge2[0]/edge2[1]))
            
                #rospy.loginfo(self.rectData['angle'])
         
                rectList.append(self.rectData)
        
        #Find the largest rect area
        rectList.sort(cmp=None, key=lambda x: x['area'], reverse=True)
        if rectList:
            self.rectData = rectList[0]
            self.rectData['detected'] = True
            
            #Draw output image 
            centerx = int(self.rectData['centroids'][0])
            centery = int(self.rectData['centroids'][1])
            contourImg = cv2.cvtColor(contourImg, cv2.cv.CV_GRAY2RGB)
            cv2.circle(contourImg, (centerx, centery), 5, (255,0,0))
            cv2.circle(out, (centerx, centery), 5, (255,255,255))
            for i in range (4):
                pt1 = (int(points[i][0]), int(points[i][1]))
                pt2 = (int(points[(i+1)%4][0]), int(points[(i+1)%4][1]))
                length = int(points[i][0]) - int(points[(i+1)%4][0])
                width = int(points[i][1]) - int(points[(i+1)%4][1])
                #print length
                #print width
                cv2.line(contourImg, pt1, pt2, (255,0,0))
                cv2.line(out, pt1, pt2, (0,0,255))
            cv2.putText(out, str(self.rectData['angle']), (30,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))
            cv2.putText(contourImg, str(self.rectData['angle']), (30,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0))
            
        else:
            self.rectData['detected'] = False 
            contourImg = cv2.cvtColor(contourImg, cv2.cv.CV_GRAY2RGB)            
              
        #return out
        return contourImg
              
    def computeCenter(self, centroid_x, centroid_y):
          x_ave = np.ave(centroid_x, None, None)
          y_ave = np.ave(centroid_y, None, None)
          return x_ave, y_ave
          
    def processImage(self, data):
        try:
            cv_image = self.rosimg2cv(data)
        except CvBridgeError, e:
            print e
        
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)   #Convert to HSV image
        hsv_image = np.array(hsv_image, dtype=np.uint8)         #Convert to numpy array
        
        centroid_image = self.findTheFlare(hsv_image)
        
        try:
            if (centroid_image != None):
                centroid_image = cv2.cv.fromarray(centroid_image)
                if (self.image_pub != None):
                    self.image_pub.publish(self.bridge.cv_to_imgmsg(centroid_image, encoding="bgr8"))
        except CvBridgeError, e:
            print e
            
if __name__ == '__main__':
    rospy.init_node('Bucket Vision', anonymous=False)
    rosRate = rospy.Rate(20)
    bkt = bucketvision(False)
    rospy.loginfo("Bucket loaded!")
    
    register()
    img = findTheBucket()
    processImage(img)    
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down Bucket")
    pass
        