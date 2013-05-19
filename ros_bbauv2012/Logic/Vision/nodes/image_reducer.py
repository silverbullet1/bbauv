#!/usr/bin/env python2
'''
Reduce image sizes and publish.
Usage: rosrun Vision image_reducer <list of topic names> [_factor:=<factor>]
'''

import roslib; roslib.load_manifest('Vision')
import rospy
from sensor_msgs.msg import Image

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sys


class ImageReducer:
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

        for topic in topics:
            outTopic = "/debug" + topic
            callback = self.reducerCallback(topic, outTopic)
            rospy.Subscriber(topic, Image, callback)


    # Returns a callback for a particular Image topic which reduces the image
    # and publishes it
    def reducerCallback(self, imageTopic, outTopic):
        publisher = rospy.Publisher(outTopic, Image)
        factor = self.factor
        def callback(rosImage):
            try:
                cvimg = self.cvbridge.imgmsg_to_cv(rosImage, desired_encoding="passthrough")
                outimg = cv2.cv.CreateMat(int(cvimg.rows * factor), int(cvimg.cols * factor), cvimg.type)
                cv2.cv.Resize(cvimg, outimg)
                publisher.publish(self.cvbridge.cv_to_imgmsg(outimg, encoding="bgr8")) #TODO: figure out actual encoding
            except CvBridgeError, e:
                print e

        return callback



# Main
if __name__ == '__main__':
    loopRateHz = 20
    rospy.init_node('image_reducer', anonymous=True)

    scaleFactor = rospy.get_param('~factor', 0.5)
    imageTopics = rospy.myargv(argv=sys.argv)[1:]

    app = ImageReducer(imageTopics, scaleFactor)

    r = rospy.Rate(loopRateHz)
    while not rospy.is_shutdown():
        key = cv2.waitKey(20)
        if key == 27: # Exit on getting the Esc key
            break

        r.sleep()

# vim: set sw=4 ts=4 expandtab:
