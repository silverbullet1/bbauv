#!/usr/bin/env python2
'''
Identify wheel (driving task)
'''

import roslib; roslib.load_manifest('Vision')
import rospy
from sensor_msgs.msg import Image, CameraInfo
from bbauv_vision_tasks.msg import wheel_task_pose
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2

class wheel_task:
    def __init__(self):
        self.cvbridge = CvBridge()

        self.cam_sub_ = rospy.Subscriber('/stereo_camera/left/image_rect_color', Image, self.ImageCB)

        self.find_times_ = 0
        self.find_times_limit_ = 30
        self.cameraInfo_initialized_ = False

        self.wheel_diameter_ = 0.38
        self.meter_pixel_ratio_ = 0.0
        self.estimated_depth_ = 0.0

        self.pipePose_pub_ = rospy.Publisher('/wheel_task_pose', wheel_task_pose)


    def ImageCB(self, image_msg, info_msg):
        if not self.cameraInfo_initialized_:
            self.camera_info_ = info_msg
            self.cameraInfo_initialized_ = True

        try:
            iplimg = self.cvbridge.imgmsg_to_cv(image_msg, image_msg.encoding)
            cvimg = np.array(iplimg, dtype=np.uint8)
        except CvBridgeError, e:
            rospy.logerr('cvbridge exception: ' + str(e))
            return

        hsv_image = cv2.cvtColor(cvimg, cv2.cv.CV_BGR2HSV)
        hsv_clone1 = hsv_image.copy()
        hsv_clone2 = hsv_image.copy()

        yellow_image = self.yellow_threshold(hsv_image)
        red_image = self.red_threshold(hsv_clone1)
        green_image = self.green_threshold(hsv_clone2)

        task_pose = wheel_task_pose()
        #task_pose.header = iplimg.header
        self.yellow_board_detection(yellow_image, task_pose)
        self.red_wheel_detection(red_image, task_pose)
        self.green_bar_detection(green_image, task_pose)

        self.pipePose_pub_.publish(task_pose)

        cv2.imshow('green_image', green_image)
        cv2.imshow('red_image', red_image)
        cv2.imshow('yellow_image', yellow_image)
        cv2.waitKey(2)


    def threshold(self, input_image, color_min, color_max):
        output_image = cv2.inRange(input_image, color_min, color_max)

        opening_size = 1
        opening_element = cv2.getStructuringElement(cv2.MORPH_RECT, (2*opening_size+1, 2*opening_size+1))
        closing_size = 3
        closing_element = cv2.getStructuringElement(cv2.MORPH_RECT, (2*closing_size+1, 2*closing_size+1))

        output_image = cv2.morphologyEx(output_image, cv2.MORPH_OPEN, opening_element)
        output_image = cv2.morphologyEx(output_image, cv2.MORPH_CLOSE, closing_element)

        return output_image


    def yellow_threshold(self, input_image):
        color_min = np.array([15, 100, 120], np.uint8)
        color_max = np.array([60, 255, 255], np.uint8)
        return self.threshold(input_image, color_min, color_max)


    def red_threshold(self, input_image):
        color_min = np.array([1, 180, 80], np.uint8)
        color_max = np.array([10, 255, 255], np.uint8)
        return self.threshold(input_image, color_min, color_max)


    def green_threshold(self, input_image):
        color_min = np.array([40, 140, 160], np.uint8)
        color_max = np.array([60, 200, 255], np.uint8)
        return self.threshold(input_image, color_min, color_max)


    def yellow_board_detection(self, yellow_image, wheelTaskPose):
        image_copy = yellow_image.copy()
        contours = cv2.findContours(image_copy, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        contours_poly = []
        boundRotateRect = []
        for contour in contours:
            poly = cv2.approxPolyDP(contour, 3, True)
            contours_poly.append(poly)
            boundRotateRect.append(cv2.minAreaRect(poly))

        max_area = 0.0
        maxArea_serial = 0
        for i,rect in enumerate(boundRotateRect):
            rect_area = rect[1][0] * rect[1][1]
            max_area = max(max_area, rect_area)
            maxArea_serial = maxArea_serial if max_area > rect_area else i

        if max_area > 60:
            wheelTaskPose.detect_yellow_board = True


    def red_wheel_detection(self, red_image, wheelTaskPose):
        image_copy = red_image.copy()
        contours = cv2.findContours(image_copy, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        contours_poly = []
        center = []
        radius = []

        for contour in contours:
            poly = cv2.approxPolyDP(contour, 1, True)
            contours_poly.append(poly)
            ctr, rad = cv2.minEnclosingCircle(contour)
            center.append(ctr)
            radius.append(rad)

        max_radius = 0.0
        maxRadius_serial = 0
        for i,rad in enumerate(radius):
            max_radius = max(max_radius, rad)
            maxRadius_serial = maxRadius_serial if max_radius > rad else i

        if max_radius > 10:
            wheelTaskPose.detect_red_wheel = True
            diameter = 2.0 * radius[maxRadius_serial]
            self.meter_pixel_ratio_ = self.wheel_diameter_ / diameter
            self.estimated_depth_ = self.meter_pixel_ratio_ * self.camera_info_.K[0]

            wheel_center = self.imagePt_to_realPt(center[maxRadius_serial], self.meter_pixel_ratio_, self.estimated_depth_)
            wheelTaskPose.wheel_center = wheel_center
        else:
            wheelTaskPose.detect_red_wheel = False
            self.meter_pixel_ratio = 0.0
            self.estimated_depth_ = 0.0


    def green_bar_detection(self, green_image, wheelTaskPose):
        image_copy = green_image.copy()
        contours = cv2.findContours(image_copy, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        contours_poly = []
        boundRotateRect = []
        for contour in contours:
            poly = cv2.approxPolyDP(contour, 3, True)
            contours_poly.append(poly)
            boundRotateRect.append(cv2.minAreaRect(poly))

        max_area = 0.0
        maxArea_serial = 0
        for i,rect in enumerate(boundRotateRect):
            rect_area = rect[1][0] * rect[1][1]
            max_area = max(max_area, rect_area)
            maxArea_serial = maxArea_serial if max_area > rect_area else i

        if max_area > 10.0
            wheelTaskPose.detect_green_bar = True
            if wheelTaskPose.detect_red_wheel:
                bar_center = self.imagePt_to_realPt(boundRotateRect[maxArea_serial][0], self.meter_pixel_ratio_, self.estimated_depth_)
                wheelTaskPose.bar_angle = math.atan2(bar_center.y-wheelTaskPose.wheel_center.y, bar_center.x-wheelTaskPose.wheel_center.x)*180.0/math.pi


    # pose in the OpenCV convention camera-coordinate, pay attention to change into ROS coordinate
    def imagePt_to_realPt(self, imagePt, ratio, depth_z):
        realPt = Point32()
        realPt.x = ratio * (imagePt[0] - self.camera_info_.K[2])
        realPt.y = ratio * (imagePt[1] - self.camera_info_.K[5])
        realPt.z = depth_z

        return realPt


if __name__ == '__main__':
    rospy.init_node('wheel_task', anonymous=False)
    wheel_task_node = wheel_task()

    rospy.spin()

