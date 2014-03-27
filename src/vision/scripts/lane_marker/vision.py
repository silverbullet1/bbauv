import roslib; roslib.load_manifest('vision')

import math
import numpy as np
import cv2

class LaneMarkerVision:
    # Vision parameters
    hsvLoThresh = (19, 58, 0)
    hsvHiThresh = (54, 128, 255)
    minContourArea = 12000
    
    houghThreshold = 80
    houghMinLength = 60
    houghMaxGap = 10

    def init(self, com=None, debugMode=True):
        self.com = com
        self.debugMode = debugMode

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
        det = 1.0 / (u2 * v1 - u1 * v2)
        dx, dy = x2 - x1, y2 - y1
        t1 = det * (-v2 * dx + u2 * dy)
        t2 = det * (-v1 * dx + u1 * dy)
        return (t1, t2)

    # Main processing function, should return (retData, outputImg)
    def gotFrame(self, img):
        foundLines = []
        centroid = (-1, -1)
        outImg = None

        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        binImg = cv2.inRange(hsvImg, self.hsvLoThresh, self.hsvHiThresh)
        
        # Closing up gaps and remove noise with morphological ops
        erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        openEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        
        binImg = cv2.erode(binImg, erodeEl)
        binImg = cv2.dilate(binImg, dilateEl)
        binImg = cv2.morphologyEx(binImg, cv2.MORPH_OPEN, openEl)

        if self.debugMode: outImg = binImg.copy() 

        # Find large enough contours and their bounding rectangles
        scratchImg = binImg.copy()
        contours, _ = cv2.findContours(scratchImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contours = filter(lambda contour: cv2.contourArea(contour) > self.minContourArea, contours)
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
            # mask for the region
            mask = np.zeros_like(binImg, dtype=np.uint8)
            points = np.int32(cv2.cv.BoxPoints(rect))
            cv2.fillPoly(mask, [points], 255)
            rectImg = np.bitwise_and(binImg, mask)

            if self.debugMode:
                # Draw bounding rect
                for i in range(4):
                    pt1 = (points[i][0], points[i][1])
                    pt2 = (points[(i+1)%4][0], points[(i+1)%4][1])
                    cv2.line(outImg, pt1, pt2, (0,0,255), 1)

            lines = cv2.HoughLinesP(rectImg, 1, math.pi/180.0,
                                    self.houghThreshold, self.houghMinLength, self.houghMaxGap)
            # Find and fix angle
            if lines != None:
                gradients = [math.atan2(y2 - y1, x2 - x1)
                             for x1, y1, x2, y2 in lines]
                gradient = np.median(gradients)
                angle = np.rad2deg(gradient)

                foundLines.append({'pos': rect[0], 'angle': angle})

        if self.debugMode:
            for line in foundLines:
                startpt = line['pos']
                gradient = np.deg2rad(line['angle'])
                endpt = (int(startpt[0] + 100 * math.cos(gradient)),
                         int(startpt[1] + 100 * math.sin(gradient)))
                startpt = (int(startpt[0]), int(startpt[1]))
                cv2.line(outImg, startpt, endpt, (255, 0, 0), 2)

        retData = { 'foundLines': foundLines, 'centroid': centroid }
        return retData, outImg

def main():
    cv2.namedWindow("test")
    inImg = cv2.imread("test.jpg")
    detector = LaneMarkerVision()
    outImg = detector.gotFrame(inImg)
    cv2.imshow("test", outImg)
