import rospy

import math
import numpy as np
import cv2

from utils.utils import Utils
from bot_common.vision import Vision

class LaneMarkerVision:
    screen = { 'width': 640, 'height': 480 }

    # Vision parameters
    hsvLoThresh1 = (10, 0, 0)
    hsvHiThresh1 = (35, 255, 255)
    minContourArea = 3000

    yellowLoThresh = (40, 0, 0)
    yellowHiThresh = (80, 255, 255)
    minBoxArea = 3000

    ratioBound = 1.1

    houghDistRes = 2
    houghAngleRes = math.pi/180.0
    houghThreshold = 50
    houghMinLength = 40
    houghMaxGap = 10

    def __init__(self, comms=None, debugMode=True):
        self.comms = comms
        self.debugMode = debugMode

    def updateParams(self):
        params = self.comms.params
        self.hsvLoThresh1 = params['hsvLoThresh1']
        self.hsvHiThresh1 = params['hsvHiThresh1']
        self.minContourArea = params['minContourArea']

        self.yellowLoThresh = params['yellowLoThresh']
        self.yellowHiThresh = params['yellowHiThresh']
        self.minBoxArea = params['minBoxArea']

        self.ratioBound = params['ratioBound']

    # Convert line equation to vector equation
    def vectorizeLine(self, pt, angle):
        rad = angle / 180.0 * math.pi
        u = math.cos(rad)
        v = math.sin(rad)
        return (pt, (u, v))

    # Find line intersection from lines vector equation
    def findIntersection(self, line1, line2):
        ((x1, y1), (u1, v1)) = line1
        ((x2, y2), (u2, v2)) = line2
        det = 1.0 / (u2 * v1 - u1 * v2 + 0.0001)
        dx, dy = x2 - x1, y2 - y1
        t1 = det * (-v2 * dx + u2 * dy)
        #t2 = det * (-v1 * dx + u1 * dy)
        return (x1 + t1*u1, y1 + t1*v1)

    def morphology(self, img):
        # Closing up gaps and remove noise with morphological ops
        erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        closeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        img = cv2.erode(img, erodeEl)
        img = cv2.dilate(img, dilateEl)
        img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, closeEl, iterations=3)

        return img

    def morphologyBox(self, img):
        # Closing up gaps and remove noise with morphological ops
        erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))
        dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        closeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        img = cv2.erode(img, erodeEl)
        img = cv2.dilate(img, dilateEl)
        img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, closeEl, iterations=3)

        return img

    def findContourAndBound(self, img, bound=3000, bounded=True):
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_NONE)
        if bounded:
            contours = filter(lambda c: cv2.contourArea(c) > bound,
                              contours)
        return contours

    def findYellowBox(self, img):
        box = dict()
        retData = {'box': box}
        outImg = None

        img = cv2.resize(img, (self.screen['width'], self.screen['height']))
        img = Vision.enhance(img)
        img = cv2.GaussianBlur(img, (5, 5), 0)
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        binImg = cv2.inRange(hsvImg, self.yellowLoThresh, self.yellowHiThresh)
        binImg = self.morphologyBox(binImg)

        if self.debugMode:
            outImg = binImg.copy()
            outImg = cv2.cvtColor(outImg, cv2.COLOR_GRAY2BGR)

        # Find large enough contours and their bounding rectangles
        scratchImg = binImg.copy()
        contours = self.findContourAndBound(scratchImg,
                                            bound=self.minBoxArea)
        if len(contours) < 1: return retData, outImg

        # Sort the thresholded areas from largest to smallest
        largestContour = max(contours, key=cv2.contourArea)
        boxRect = cv2.minAreaRect(largestContour)
        box['centroid'] = boxRect[0]

        if self.debugMode:
            cv2.circle(outImg, (int(boxRect[0][0]), int(boxRect[0][1])),
                       5, (0, 0, 255))
            Vision.drawRect(outImg,
                            cv2.cv.BoxPoints(boxRect), color=(0, 255, 255))
            # Draw the centering rectangle
            midX = self.screen['width']/2.0
            midY = self.screen['height']/2.0
            maxDeltaX = self.screen['width']*0.03
            maxDeltaY = self.screen['height']*0.03
            cv2.rectangle(outImg,
                          (int(midX-maxDeltaX), int(midY-maxDeltaY)),
                          (int(midX+maxDeltaX), int(midY+maxDeltaY)),
                          (0, 255, 0), 2)

        return retData, outImg

    def findLane(self, img):
        foundLines = []
        centroid = [0, 0]
        outImg = None
        retData = { 'foundLines': foundLines, 'centroid': centroid }

        img = cv2.resize(img, (self.screen['width'], self.screen['height']))
        img = Vision.enhance(img)
        img = cv2.GaussianBlur(img, (5, 5), 0)
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        binImg = cv2.inRange(hsvImg, self.hsvLoThresh1, self.hsvHiThresh1)
        binImg = self.morphology(binImg)

        if self.debugMode:
            outImg = binImg.copy()
            outImg = cv2.cvtColor(outImg, cv2.COLOR_GRAY2BGR)

        # Find large enough contours and their bounding rectangles
        scratchImg = binImg.copy()
        contours = self.findContourAndBound(scratchImg,
                                            bound=self.minContourArea)
        if len(contours) < 1: return retData, outImg
        # Sort the thresholded areas from largest to smallest
        sorted(contours, key=cv2.contourArea, reverse=True)

        contourRects = [cv2.minAreaRect(contour) for contour in contours]
        # Find the centroid from averaging all the contours
        for contour in contours:
            mu = cv2.moments(contour, False)
            muArea = mu['m00']
            centroid[0] += mu['m10'] / muArea
            centroid[1] += mu['m01'] / muArea

        centroid[0] /= len(contours)
        centroid[1] /= len(contours)


        # Find lines in each bounded rectangle region and find angle
        for rect in contourRects:
            # Mask for the region
            #mask = np.zeros_like(binImg, dtype=np.uint8)
            points = np.int32(cv2.cv.BoxPoints(rect))
            #cv2.fillPoly(mask, [points], 255)
            #rectImg = np.bitwise_and(binImg, mask)

            #Find the lane heading
            edge1 = points[1] - points[0]
            edge2 = points[2] - points[1]

            edge1Len = cv2.norm(edge1)
            edge2Len = cv2.norm(edge2)
            ratio = edge1Len/edge2Len if edge1Len > edge2Len else edge2Len/edge1Len
            if ratio < self.ratioBound:
                continue

            #Choose the vertical edge
            if edge1Len > edge2Len:
                rectAngle = math.degrees(math.atan2(edge1[1], edge1[0]))
            else:
                rectAngle = math.degrees(math.atan2(edge2[1], edge2[0]))

            # Draw bounding rect
            if self.debugMode:
                Vision.drawRect(outImg, points)

            #lines = cv2.HoughLinesP(rectImg,
            #                        self.houghDistRes, self.houghAngleRes,
            #                        self.houghThreshold,
            #                        self.houghMinLength,
            #                        self.houghMaxGap)
            ## Find and fix angle
            #if lines != None:
            #    #for line in lines[0]:
            #    #    cv2.line(outImg,
            #    #            (line[0], line[1]), (line[2], line[3]),
            #    #            (0,255,0), 1)
            #    gradients = [math.atan2(y2 - y1, x2 - x1)
            #                 for x1, y1, x2, y2 in lines[0]]
            #    gradient = np.median(gradients)
            #    angle = np.rad2deg(gradient)

            foundLines.append({'pos': rect[0], 'angle': rectAngle})
            #                   'testAngle': angle})

        if len(foundLines) >= 2 and self.comms.expectedLanes == 2:
            # If there are 2 lines, find their intersection and adjust angle
            l1 = self.vectorizeLine(foundLines[0]['pos'],
                                    foundLines[0]['angle'])
            l2 = self.vectorizeLine(foundLines[1]['pos'],
                                    foundLines[1]['angle'])
            crossPt = self.findIntersection(l1, l2) # intersection b/w l1 & l2

            if self.debugMode:
                cv2.circle(outImg, (int(crossPt[0]), int(crossPt[1])),
                           3, (0, 255, 0))
            foundLines[0]['angle'] = np.rad2deg(math.atan2(l1[0][1]-crossPt[1],
                                                           l1[0][0]-crossPt[0]))
            foundLines[1]['angle'] = np.rad2deg(math.atan2(l2[0][1]-crossPt[1],
                                                           l2[0][0]-crossPt[0]))
            retData['crossPoint'] = crossPt

            # Figure out which lane marker is left or right
            left = Utils.normAngle(foundLines[0]['angle'])
            right = Utils.normAngle(foundLines[1]['angle'])
            if (not ((right-left > 0 and abs(right-left) < 180) or
                     (right-left < 0 and abs(right-left) > 180))):
                foundLines[0], foundLines[1] = foundLines[1], foundLines[0]

            if self.debugMode:
                startPt = (int(foundLines[0]['pos'][0])-70,
                           int(foundLines[0]['pos'][1]))
                cv2.putText(outImg, "left", startPt,
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 1)

        elif len(foundLines) >= 1:
            centroid = foundLines[0]['pos']
            # Otherwise adjust to the angle closest to input heading
            lineAngle = foundLines[0]['angle']
            adjustAngle = Utils.normAngle(self.comms.curHeading +
                                          Utils.toHeadingSpace(lineAngle))
            if 90 < abs(Utils.normAngle(self.comms.inputHeading) - adjustAngle) < 270:
                foundLines[0]['angle'] = Utils.invertAngle(lineAngle)

        if self.debugMode:
            # Draw the centroid
            cv2.circle(outImg,
                       (int(centroid[0]), int(centroid[1])),
                       3, (0, 0, 255))
            # Draw vector line and angle
            for line in foundLines:
                startpt = line['pos']
                gradient = np.deg2rad(line['angle'])
                endpt = (int(startpt[0] + 100 * math.cos(gradient)),
                         int(startpt[1] + 100 * math.sin(gradient)))
                startpt = (int(startpt[0]), int(startpt[1]))
                angleStr = "{0:.2f}".format(Utils.toHeadingSpace(line['angle']))

                cv2.line(outImg, startpt, endpt, (255, 0, 0), 2)
                cv2.circle(outImg, startpt, 3, (0, 0, 255), 1)
                cv2.putText(outImg, angleStr, startpt,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            # Draw the centering rectangle
            midX = self.screen['width']/2.0
            midY = self.screen['height']/2.0
            maxDeltaX = self.screen['width']*0.03
            maxDeltaY = self.screen['height']*0.03
            cv2.rectangle(outImg,
                          (int(midX-maxDeltaX), int(midY-maxDeltaY)),
                          (int(midX+maxDeltaX), int(midY+maxDeltaY)),
                          (0, 255, 0), 2)

        return retData, outImg

    # Main processing function, should return (retData, outputImg)
    def gotFrame(self, img):
        if self.comms.detectingBox:
            return self.findYellowBox(img)
        else:
            return self.findLane(img)

def main():
    rospy.init_node("lane_marker_vision")
    cv2.namedWindow("test")
    from comms import Comms
    inImg = cv2.imread("lane_marker/test1.jpg")
    detector = LaneMarkerVision(Comms())
    _, outImg = detector.gotFrame(inImg)
    if outImg is not None: cv2.imshow("test", outImg)
    cv2.waitKey()
