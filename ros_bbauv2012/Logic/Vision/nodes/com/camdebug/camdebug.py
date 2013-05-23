#!/usr/bin/python2

import roslib; roslib.load_manifest('Vision')
import rospy
import cv2
import cv2.cv as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

'''
Helper functions
'''
def cv2array(im):
    depth2dtype = {
        cv.IPL_DEPTH_8U: 'uint8',
        cv.IPL_DEPTH_8S: 'int8',
        cv.IPL_DEPTH_16U: 'uint16',
        cv.IPL_DEPTH_16S: 'int16',
        cv.IPL_DEPTH_32S: 'int32',
        cv.IPL_DEPTH_32F: 'float32',
        cv.IPL_DEPTH_64F: 'float64',
    }

    arrdtype=im.depth
    a = np.fromstring(
             im.tostring(),
             dtype=depth2dtype[im.depth],
             count=im.width*im.height*im.nChannels)
    a.shape = (im.height,im.width,im.nChannels)
    return a

def array2cv(a):
    dtype2depth = {
        'uint8':   cv.IPL_DEPTH_8U,
        'int8':    cv.IPL_DEPTH_8S,
        'uint16':  cv.IPL_DEPTH_16U,
        'int16':   cv.IPL_DEPTH_16S,
        'int32':   cv.IPL_DEPTH_32S,
        'float32': cv.IPL_DEPTH_32F,
        'float64': cv.IPL_DEPTH_64F,
    }
    try:
        nChannels = a.shape[2]
    except:
        nChannels = 1
    cv_im = cv.CreateImageHeader((a.shape[1],a.shape[0]),
              dtype2depth[str(a.dtype)],
              nChannels)
    cv.SetData(cv_im, a.tostring(),
               a.dtype.itemsize*nChannels*a.shape[1])
    return cv_im


'''
Debugger for our computer vision algos.
'''
class CamDebug():
    def __init__(self, nodeName, debugOn=True):
        self.nodeName = nodeName
        self.debugOn = debugOn
        self.topics = {}

        self.cvbridge = CvBridge()

    '''
    Publish an image to a stream (useful for debugging).
    Expects image to be a numpy array.
    '''
    def publishImage(self, topicName, image):
        if not self.debugOn:
            return

        if topicName not in self.topics:
            fullTopicName = '/' + self.nodeName + '/' + topicName
            self.topics[topicName] = rospy.Publisher(fullTopicName, Image)

        publisher = self.topics[topicName]
        try:
            outImg = array2cv(image)
            publisher.publish(self.cvbridge.cv_to_imgmsg(outImg))
        except CvBridgeError, e:
            print e

# vim: set sw=4 ts=4 expandtab:
