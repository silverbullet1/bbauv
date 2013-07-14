#!/usr/bin/env python2
'''
Reduce image sizes and publish.
Usage: rosrun Vision image_reducer <list of topic names> [_factor:=<factor>]
'''

import roslib; roslib.load_manifest('Vision')
import rospy
from sensor_msgs.msg import Image
from bbauv_msgs.msg import compass_data

import math
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sys


class ImageDebugger:
#    # Convert a ROS Image to the Numpy matrix used by cv2 functions
#    def rosimg2cv(self, ros_image):
#        # Convert from ROS Image to old OpenCV image
#        frame = self.cvbridge.imgmsg_to_cv(ros_image, ros_image.encoding)
#        # Convert from old OpenCV image to Numpy matrix
#        return np.array(frame, dtype=np.uint8) #TODO: find out actual dtype


    def __init__(self, topics, factor):
        self.cvbridge = CvBridge()
        self.topics = topics
        self.factor = factor

        self.yaw   = 0
        self.pitch = 0
        self.roll  = 0

        for topic in topics:
            outTopic = "/debug" + topic
            callback = self.reducerCallback(topic, outTopic, showReticle=('/left/' in topic))
            rospy.Subscriber(topic, Image, callback)

        rospy.Subscriber('/euler', compass_data, self.gotAttitude)


    def gotAttitude(self, msg):
        self.yaw   = msg.yaw
        self.pitch = msg.pitch
        self.roll  = msg.roll


    # Returns a callback for a particular Image topic which reduces the image
    # and publishes it
    def reducerCallback(self, imageTopic, outTopic, showReticle=False):
        publisher = rospy.Publisher(outTopic, Image)
        factor = self.factor
        def plainCallback(rosImage):
            try:
                cvimg = self.cvbridge.imgmsg_to_cv(rosImage, desired_encoding="passthrough")
                outimg = cv2.cv.CreateMat(int(cvimg.rows * factor), int(cvimg.cols * factor), cvimg.type)
                cv2.cv.Resize(cvimg, outimg)
                publisher.publish(self.cvbridge.cv_to_imgmsg(outimg, encoding="bgr8")) #TODO: figure out actual encoding

            except CvBridgeError, e:
                print e

        def reticleCallback(rosImage):
            try:
                cvimg = self.cvbridge.imgmsg_to_cv(rosImage, desired_encoding="passthrough")
                cvimg = np.array(cvimg, dtype=np.uint8)

                cvimg = cv2.resize(cvimg, (0, 0), None, factor, factor)
                self.drawReticle(cvimg)

                outimg = cv2.cv.fromarray(cvimg)
                publisher.publish(self.cvbridge.cv_to_imgmsg(outimg, encoding="bgr8")) #TODO: figure out actual encoding
            except CvBridgeError, e:
                print e

        return (reticleCallback if showReticle else plainCallback)


    def drawReticle(self, img):
        yaw, pitch, roll = self.yaw, self.pitch, self.roll

        DEGREE_PIXEL_RATIO = 0.1
        H_DEGREE_PIXEL_RATIO = 0.3
        height, width, _ = img.shape
        colour = (0, 90, 0)
        pitch_start, pitch_end = 40, height-40
        yaw_start, yaw_end = 40, width-40

        mid_x, mid_y = width/2, height/2

        # Draw indicators
        cv2.line(img, (mid_x-70, mid_y), (mid_x-50, mid_y), (0, 0, 255), 1)
        cv2.line(img, (mid_x+50, mid_y), (mid_x+70, mid_y), (0, 0, 255), 1)
        cv2.line(img, (mid_x, 33), (mid_x-5, 38), (0, 0, 255), 1)
        cv2.line(img, (mid_x, 33), (mid_x+5, 38), (0, 0, 255), 1)
        cv2.line(img, (mid_x, pitch_end+13), (mid_x-5, pitch_end+10), (0, 0, 255), 1)
        cv2.line(img, (mid_x, pitch_end+13), (mid_x+5, pitch_end+10), (0, 0, 255), 1)

        # Multiply by 10 to work in integers
        origin_pitch = int(10 * (DEGREE_PIXEL_RATIO * (mid_y-pitch_start) + pitch))
        # Round to multiple of 25 lower than this
        BASE = 25
        closest_pitch = int(BASE * round(float(origin_pitch)/BASE))
        closest_pitch -= BASE if closest_pitch > origin_pitch else 0

        pitch_y = pitch_start + int((origin_pitch - closest_pitch) / (10 * DEGREE_PIXEL_RATIO))
        pitch_inc = int(BASE / (10 * DEGREE_PIXEL_RATIO))
        current_pitch = closest_pitch

        # Draw horizontal lines
        while pitch_y < pitch_end:
            thickness = 1
            offset = 6
            if current_pitch % 50 == 0:
                offset = 10
            if current_pitch % 100 == 0:
                offset = 18
     
            pt1 = (mid_x-offset, pitch_y)
            pt2 = (mid_x+offset, pitch_y)
            cv2.line(img, pt1, pt2, colour, thickness)

            if current_pitch % 100 == 0:
                txt = str(abs(current_pitch)/10)
                (txt_w, txt_h), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_PLAIN, 0.8, 1)
                pt = (mid_x-offset-txt_w-2, pitch_y + txt_h/2)
                cv2.putText(img, txt, pt, cv2.FONT_HERSHEY_PLAIN, 0.8, colour)

                pt = (mid_x+offset+2, pitch_y + txt_h/2)
                cv2.putText(img, txt, pt, cv2.FONT_HERSHEY_PLAIN, 0.8, colour)
     
            current_pitch -= BASE
            pitch_y += pitch_inc

        # Draw arc
        angle = int(180 - roll)
        cv2.ellipse(img, (mid_x, 140), (180, 120), angle, 75, 105, colour)
        arcpts = cv2.ellipse2Poly((mid_x, 140), (180, 120), angle, 75, 105, 15)
        for i, pt in enumerate(arcpts):
            disp_angle = (i-1) * 15
            txt = str(abs(disp_angle))
            txt_angle = np.deg2rad(-roll - 90 + disp_angle)
            (txt_w, txt_h), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_PLAIN, 0.8, 1)
            txt_x = int(pt[0] + 6 * math.cos(txt_angle)) - txt_w/2
            txt_y = int(pt[1] + 6 * math.sin(txt_angle))

            cv2.putText(img, txt, (txt_x, txt_y), cv2.FONT_HERSHEY_PLAIN, 0.8, colour)
            cv2.ellipse(img, (pt[0], pt[1]), (1,1), 0, 0, 360, colour)

        # Draw horizontal band
        CARDINALS = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']

        origin_yaw = int(-H_DEGREE_PIXEL_RATIO * (mid_x-yaw_start) + yaw)
        # Round to multiple of 5 greater than this
        H_BASE = 5
        closest_yaw = int(H_BASE * round(float(origin_yaw)/H_BASE))
        closest_yaw += H_BASE if closest_yaw < origin_yaw else 0

        yaw_x = 5 + yaw_start + int((closest_yaw - origin_yaw) / float(H_DEGREE_PIXEL_RATIO))
        yaw_inc = int(H_BASE / float(H_DEGREE_PIXEL_RATIO))
        current_yaw = closest_yaw

        yaw_bottom = pitch_end + 30

        while yaw_x < yaw_end:
            thickness = 1
            offset = 3 if current_yaw % 15 else 6
     
            pt1 = (yaw_x, yaw_bottom)
            pt2 = (yaw_x, yaw_bottom - offset)
            cv2.line(img, pt1, pt2, colour, thickness)

            if current_yaw % 15 == 0:
                disp_yaw = current_yaw if current_yaw >= 0 else current_yaw + 360
                disp_yaw = disp_yaw if current_yaw < 360 else current_yaw - 360
                txt = str(disp_yaw) if current_yaw % 45 else CARDINALS[disp_yaw / 45]
                (txt_w, txt_h), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_PLAIN, 0.8, 1)
                pt = (yaw_x-txt_w/2, yaw_bottom - txt_h)
                cv2.putText(img, txt, pt, cv2.FONT_HERSHEY_PLAIN, 0.8, colour)
     
            current_yaw += H_BASE
            yaw_x += yaw_inc



# Main
if __name__ == '__main__':
    loopRateHz = 20
    rospy.init_node('image_debugger', anonymous=True)

    scaleFactor = rospy.get_param('~factor', 0.5)
    imageTopics = ['/stereo_camera/left/image_rect_color', '/stereo_camera/right/image_rect_color', '/bottomcam/camera/image_rect_color']

    app = ImageDebugger(imageTopics, scaleFactor)

    r = rospy.Rate(loopRateHz)
    while not rospy.is_shutdown():
        key = cv2.waitKey(20)
        if key == 27: # Exit on getting the Esc key
            break

        r.sleep()

# vim: set sw=4 ts=4 expandtab:
