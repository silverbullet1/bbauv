import math
import numpy as np
import cv2

from utils.utils import Utils
from bot_common.vision import Vision


class PickupVision:
    SITE = 0
    SAMPLES = 1
    BOX = 2

    screen = {'width': 640, 'height': 480}

    # Vision parameters
    greenLoThresh = (35, 0, 0)
    greenHiThresh = (70, 255, 255)

    redLoThresh1 = (1, 0, 0)
    redHiThresh1 = (25, 255, 255)
    redLoThresh2 = (160, 0, 0)
    redHiThresh2 = (180, 255, 255)

    yellowLoThresh = (25, 0, 0)
    yellowHiThresh = (50, 255, 255)

    minContourArea = 5000
    minSiteArea = 10000

    def __init__(self, comms=None, debugMode=True):
        self.comms = comms
        self.debugMode = debugMode

    def updateParams(self):
        self.greenLoThresh = self.comms.params['greenLoThresh']
        self.greenHiThresh = self.comms.params['greenHiThresh']
        self.redLoThresh1 = self.comms.params['redLoThresh1']
        self.redHiThresh1 = self.comms.params['redHiThresh1']
        self.redLoThresh2 = self.comms.params['redLoThresh2']
        self.redHiThresh2 = self.comms.params['redHiThresh2']
        self.yellowLoThresh = self.comms.params['yellowLoThresh']
        self.yellowHiThresh = self.comms.params['yellowHiThresh']
        self.minSiteArea = self.comms.params['minSiteArea']
        self.minContourArea =self.comms.params['minContourArea']

    def morphology(self, img):
        # Closing up gaps and remove noise with morphological ops
        erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))
        dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
        openEl = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

        img = cv2.erode(img, erodeEl)
        img = cv2.dilate(img, dilateEl)
        img = cv2.morphologyEx(img, cv2.MORPH_OPEN, openEl)

        return img

    def morphologySamples(self, img):
        # Closing up gaps and remove noise with morphological ops
        erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, (13, 13))
        openEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        img = cv2.erode(img, erodeEl)
        img = cv2.dilate(img, dilateEl)
        img = cv2.morphologyEx(img, cv2.MORPH_OPEN, openEl)

        return img

    def findContourAndBound(self, img, bounded=True, bound=0):
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_NONE)
        if bounded:
            contours = filter(lambda c: cv2.contourArea(c) > bound, contours)

        return contours

    # Main processing function, should return (retData, outputImg)
    def gotFrame(self, img):
        outImg = None
        samples = list()
        site = dict()
        box = dict()
        rval = {'samples': samples, 'site': site, 'box': box}

        img = cv2.resize(img, (self.screen['width'], self.screen['height']))
        img = Vision.enhance(img)
        img = cv2.GaussianBlur(img, (5, 5), 0)
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        if self.comms.visionMode == PickupVision.SAMPLES:
            #binImg = cv2.inRange(hsvImg, self.greenLoThresh, self.greenHiThresh)
            binImg = cv2.inRange(hsvImg, self.redLoThresh1, self.redHiThresh1)
            binImg |= cv2.inRange(hsvImg, self.redLoThresh2, self.redHiThresh2)

            binImg = self.morphologySamples(binImg)

            if self.debugMode:
                outImg = cv2.cvtColor(binImg.copy(), cv2.COLOR_GRAY2BGR)

            contours = self.findContourAndBound(binImg.copy(), bounded=True,
                                                bound=self.minContourArea)
            sorted(contours, key=cv2.contourArea, reverse=True)
            for contour in contours:
                # Find the center of each contour
                rect = cv2.minAreaRect(contour)
                centroid = rect[0]

                # Find the orientation of each contour
                points = np.int32(cv2.cv.BoxPoints(rect))
                edge1 = points[1] - points[0]
                edge2 = points[2] - points[1]

                if cv2.norm(edge1) > cv2.norm(edge2):
                    angle = math.degrees(math.atan2(edge1[1], edge1[0]))
                else:
                    angle = math.degrees(math.atan2(edge2[1], edge2[0]))

                if 90 < abs(Utils.normAngle(self.comms.curHeading) -
                            Utils.normAngle(angle)) < 270:
                    angle = Utils.invertAngle(angle)

                samples.append({'centroid': centroid, 'angle': angle,
                                'area': cv2.contourArea(contour)})

            if self.debugMode:
                # Draw the centroid and orientation of each contour
                for sample in samples:
                    cv2.circle(outImg, (int(centroid[0]), int(centroid[1])),
                               5, (0, 0, 255))
                    startpt = centroid
                    gradient = np.deg2rad(angle)
                    endpt = (int(startpt[0] + 100 * math.cos(gradient)),
                             int(startpt[1] + 100 * math.sin(gradient)))
                    startpt = (int(startpt[0]), int(startpt[1]))
                    cv2.line(outImg, startpt, endpt, (255, 0, 0), 2)
        else:
            binImg = cv2.inRange(hsvImg,
                                 self.yellowLoThresh, self.yellowHiThresh)
            binImg = self.morphology(binImg)

            if self.debugMode:
                outImg = cv2.cvtColor(binImg.copy(), cv2.COLOR_GRAY2BGR)

            contours = self.findContourAndBound(binImg.copy(), bounded=True,
                                                bound=self.minSiteArea)
            if len(contours) > 0:
                largestContour = max(contours, key=cv2.contourArea)
                rect = cv2.minAreaRect(largestContour)
                centroid = rect[0]
                site['centroid'] = centroid

                # Find the orientation of each contour
                points = np.int32(cv2.cv.BoxPoints(cv2.minAreaRect(largestContour)))
                edge1 = points[1] - points[0]
                edge2 = points[2] - points[1]

                if cv2.norm(edge1) > cv2.norm(edge2):
                    angle = math.degrees(math.atan2(edge1[1], edge1[0]))
                else:
                    angle = math.degrees(math.atan2(edge2[1], edge2[0]))

                if 90 < abs(Utils.normAngle(self.comms.curHeading) -
                            Utils.normAngle(angle)) < 270:
                    angle = Utils.invertAngle(angle)

                site['angle'] = angle

                if self.debugMode:
                    points = cv2.cv.BoxPoints(cv2.minAreaRect(largestContour))
                    Vision.drawRect(outImg, points)
                    cv2.circle(outImg, (int(centroid[0]), int(centroid[1])),
                               5, (0, 0, 255))

        if self.debugMode:
            # Draw the aiming rectangle
            midX = self.screen['width']/2.0
            midY = self.screen['height']/2.0
            maxDeltaX = self.screen['width']*0.03
            maxDeltaY = self.screen['height']*0.03
            cv2.rectangle(outImg,
                          (int(midX-maxDeltaX), int(midY-maxDeltaY)),
                          (int(midX+maxDeltaX), int(midY+maxDeltaY)),
                          (0, 255, 0), 1)

        return rval, outImg


def main():
    import rospy
    rospy.init_node("pickup_vision")
    cv2.namedWindow("output")
    image = cv2.imread("green_cheese.png")
    from comms import Comms
    visionFilter = PickupVision(Comms())
    _, outImg = visionFilter.gotFrame(image)
    cv2.imshow("output", outImg)
    cv2.waitKey()
