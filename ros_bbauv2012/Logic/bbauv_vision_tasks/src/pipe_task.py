#!/usr/bin/env python2
'''
Identify pipe (pizza delivery task)
'''

import roslib; roslib.load_manifest('Vision')
import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point32
from bbauv_vision_tasks.msg import pipe_pose
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2

class pipe_task:
    def __init__(self):
        self.cvbridge = CvBridge()

        self.cam_sub_ = rospy.Subscriber('camera/image_rect_color', Image, self.ImageCB)

        self.cameraInfo_initialized_ = False
        self.pipe_length_total_ = 0.305
        self.pipe_length_little_ = 0.0
        self.find_times_ = 0
        self.find_times_limit_ = 30

        self.pipePose_pub_ = rospy.Publisher('/pipe_pose', pipe_pose)


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
        color_min = np.array([20, 10, 20], np.uint8)
        color_max = np.array([90, 50, 100], np.uint8)

        pipe_orange_threshold = cv2.inRange(hsv_image, color_min, color_max)

        opening_size = 1
        opening_element = cv2.getStructuringElement(cv2.MORPH_RECT, (2*opening_size+1, 2*opening_size+1))
        closing_size = 5
        closing_element = cv2.getStructuringElement(cv2.MORPH_RECT, (2*closing_size+1, 2*closing_size+1))

        pipe_orange_threshold = cv2.morphologyEx(pipe_orange_threshold, cv2.MORPH_OPEN, opening_element)
        pipe_orange_threshold = cv2.morphologyEx(pipe_orange_threshold, cv2.MORPH_CLOSE, closing_element)

        cv2.imshow('raw_threshold', pipe_orange_threshold)

        white_pixel_num = np.count_nonzero(pipe_orange_threshold)

        copy_for_contour = pipe_orange_threshold.copy()

        contours = cv2.findContours(copy_for_contour, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for i in range(1, len(contours)):
            cv2.line(pipe_orange_threshold, contours[i-1][0], contours[i][0], 255, 2)

        cv2.imshow('pipe_outer', pipe_orange_threshold)

        copy_for_contour2 = pipe_orange_threshold.copy()
        single_contour = cv2.findContours(copy_for_contour2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        pipe_skeleton_pose = pipe_pose()
        #pipe_skeleton_pose.header = iplimg.header

        if white_pixel_num > 200:
            minArea_rectangle = cv2.minAreaRect(single_contour[0])
            color_rect = cv2.cvtColor(pipe_orange_threshold, cv2.cv.CV_GRAY2BGR)

            rect_points = cv2.cv.BoxPoints(minArea_rectangle)
            for j in range(4):
                cv2.line(color_rect, rect_points[j], rect_points[(j+1)%4], (255,0,0), 3, 8)

            pipe_skeleton_pose.detect_pipe, lowest_pt, second_lowest_pt = self.Calc_pose(rect_points, pipe_skeleton_pose)
            line(color_rect, lowest_pt, second_lowest_pt, (0,0,255), 3, 8)

            cv2.imshow('color_rect', color_rect)

        if pipe_skeleton_pose.detect_pipe:
            self.find_times_ += 1
        else:
            self.find_times_ -= 1

        pipe_skeleton_pose.detection_stable = self.find_times_ > self.find_times_limit_

        self.pipePose_pub_.publish(pipe_skeleton_pose)

        cv2.imshow('orange_threshold', pipe_orange_threshold)
        cv2.imshow('pipe_outer', pipe_orange_threshold)
        cv2.waitKey(2)


    def Calc_pose(self, rect_points, pipeFrame_pose):
        dist1 = math.sqrt((rect_points[0][0] - rect_points[1][0])*(rect_points[0][0] - rect_points[1][0])+(rect_points[0][1] - rect_points[1][1])*(rect_points[0][1] - rect_points[1][1]))
        dist2 = math.sqrt((rect_points[2][0] - rect_points[1][0])*(rect_points[2][0] - rect_points[1][0])+(rect_points[0][1] - rect_points[1][1])*(rect_points[0][1] - rect_points[1][1]))
        longer_line, shorter_line = (dist1, dist2) if dist1>=dist2 else (dist2, dist1)
        if shorter_line < 0.001:
            return False, (0,0), (0,0)
        if longer_line/shorter_line > 3.0:
            return False, (0,0), (0,0)

        points_border_image = 0
        for i in range(4):
            if rect_points[i][0]<2.0 or rect_points[i][1]<2.0:
                points_border_image += 1
        if points_border_image >= 3:
            return False, (0,0), (0,0)

        # Find the lowest 2 points in the image, to serve as the two ends of the lowest pipeline bar
        if rect_points[0][1] > rect_points[1][1]:
            lowest_point = rect_points[0]
            second_lowest_point = rect_points[1]
        else:
            lowest_point = rect_points[1]
            second_lowest_point = rect_points[0]
        if rect_points[2][1] > lowest_point[1]:
            second_lowest_point = lowest_point
            lowest_point = rect_points [2]
        elif rect_points[2][1] > second_lowest_point[1]:
            second_lowest_point = rect_points[2]
        if rect_points[3][1] > lowest_point[1]:
            second_lowest_point = lowest_point
            lowest_point = rect_points [3]
        elif rect_points[3][1] > second_lowest_point[1]:
            second_lowest_point = rect_points[3]

        length_pixel_ratio = self.pipe_length_total_ / longer_line
        estimated_depth = length_pixel_ratio * self.camera_info_.K[0]

        realPt1 = self.imagePt_to_realPt(lowest_point, length_pixel_ratio, estimated_depth)
        realPt2 = self.imagePt_to_realPt(second_lowest_point, length_pixel_ratio, estimated_depth)
        if realPt1.y > realPt2.y:
            realPt1, realPt2 = realPt2, realPt1

        pipeFrame_pose.points.append(realPt1)
        pipeFrame_pose.points.append(realPt2)

        return True, lowest_point, second_lowest_point


    def imagePt_to_realPt(self, imagePt, ratio, depth_z):
        realPt = Point32()
        realPt.x = ratio * (imagePt[0] - self.camera_info_.K[2])
        realPt.y = ratio * (imagePt[1] - self.camera_info_.K[5])
        realPt.z = depth_z

        return realPt


if __name__ == '__main__':
    rospy.init_node('pipe_task', anonymous=False)
    pipe_task_node = pipe_task()

    rospy.spin()

