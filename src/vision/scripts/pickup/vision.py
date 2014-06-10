import math
import numpy as np
import cv2

from utils.utils import Utils

class PickupVision:
    screen = { 'width': 640, 'height': 480 }

    # Vision parameters
    greenLoThresh = (35, 0, 0)
    greenHiThresh = (70, 255, 255)
    hsvLoThresh2 = (165, 0, 0)
    hsvHiThresh2 = (180, 255, 255)
    minContourArea = 5000

    def __init__(self, comms=None, debugMode=True):
        self.comms = comms
        self.debugMode = debugMode

    def morphology(self, img):
        # Closing up gaps and remove noise with morphological ops
        erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
        openEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        img = cv2.erode(img, erodeEl)
        img = cv2.dilate(img, dilateEl)
        img = cv2.morphologyEx(img, cv2.MORPH_OPEN, openEl)

        return img

    def findContourAndBound(self, img, bounded=True):
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_NONE)
        if bounded:
            contours = filter(lambda c: cv2.contourArea(c) > self.minContourArea,
                              contours)
        return contours

    def enhance(self, img):
        enhancedImg = cv2.GaussianBlur(img, ksize=(0, 0), sigmaX=10)
        enhancedImg = cv2.addWeighted(img, 2.5, enhancedImg, -1.5, 0)
        return enhancedImg

    # Main processing function, should return (retData, outputImg)
    def gotFrame(self, img):
        outImg = None
        centroids = list()
        angles = list()
        rval = {'centroids':centroids, 'angles': angles};

        img = cv2.resize(img, (self.screen['width'], self.screen['height']))
        img = self.enhance(img)
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        binImg = cv2.inRange(hsvImg, self.greenLoThresh, self.greenHiThresh)
        #binImg |= cv2.inRange(hsvImg, self.hsvLoThresh2, self.hsvHiThresh2)

        if self.debugMode:
            outImg = cv2.cvtColor(binImg.copy(), cv2.COLOR_GRAY2BGR)
            # Draw the aiming rectangle
            midX = self.screen['width']/2.0
            midY = self.screen['height']/2.0
            maxDeltaX = self.screen['width']*0.03
            maxDeltaY = self.screen['height']*0.03
            cv2.rectangle(outImg,
                          (int(midX-maxDeltaX), int(midY-maxDeltaY)),
                          (int(midX+maxDeltaX), int(midY+maxDeltaY)),
                          (0, 255, 0), 1)

        contours = self.findContourAndBound(binImg.copy(), bounded=True)
        sorted(contours, key=cv2.contourArea, reverse=True)
        for contour in contours:
            # Find the center of each contour
            moment = cv2.moments(contour, False)
            centroid = (moment['m10']/moment['m00'],
                        moment['m01']/moment['m00'])
            centroids.append(centroid)

            # Find the orientation of each contour
            points = np.int32(cv2.cv.BoxPoints(cv2.minAreaRect(contour)))
            edge1 = points[1] - points[0]
            edge2 = points[2] - points[1]

            if cv2.norm(edge1) > cv2.norm(edge2):
                angle = math.degrees(math.atan2(edge1[1], edge1[0]))
            else:
                angle = math.degrees(math.atan2(edge2[1], edge2[0]))

            if 90 < abs(Utils.normAngle(self.comms.curHeading) -
                        Utils.normAngle(angle)) < 270:
                angle = Utils.invertAngle(angle)

            angles.append(angle)

            # Draw the centroid and orientation of each contour
            for centroid in centroids:
                cv2.circle(outImg, (int(centroid[0]),int(centroid[1])),
                           5, (0, 0, 255))
                startpt = centroid 
                gradient = np.deg2rad(angle)
                endpt = (int(startpt[0] + 100 * math.cos(gradient)),
                         int(startpt[1] + 100 * math.sin(gradient)))
                startpt = (int(startpt[0]), int(startpt[1]))
                cv2.line(outImg, startpt, endpt, (255, 0, 0), 2)

        if self.debugMode:
            # Draw the centering rectangle
            midX = self.screen['width']/2.0
            midY = self.screen['height']/2.0
            maxDeltaX = self.screen['width']*0.03
            maxDeltaY = self.screen['height']*0.03
            cv2.rectangle(outImg,
                          (int(midX-maxDeltaX), int(midY-maxDeltaY)),
                          (int(midX+maxDeltaX), int(midY+maxDeltaY)),
                          (0, 255, 0), 2)

        return rval, outImg

def main():
    cv2.namedWindow("output")
    image = cv2.imread("green_cheese.png")
    visionFilter = PickupVision()
    _, outImg = visionFilter.gotFrame(image)
    cv2.imshow("output", outImg)
    cv2.waitKey()
