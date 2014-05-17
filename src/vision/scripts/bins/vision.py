import cv2

from bot_common.vision import Vision

class BinsVision:
    screen = { 'width': 640, 'height': 480 }

    # Vision parameters
    hsvLoThresh1 = (0, 0, 0)
    hsvHiThresh1 = (20, 255, 255)
    hsvLoThresh2 = (165, 0, 0)
    hsvHiThresh2 = (180, 255, 255)
    minContourArea = 5000

    # Parameters for gray-scale thresholding
    upperThresh = 70
    areaThresh = 10000

    def __init__(self, comms=None, debugMode=True):
        self.comms = comms
        self.debugMode = debugMode

    def morphology(self, img):
        # Closing up gaps and remove noise with morphological ops
        erodeEl = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        dilateEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        openEl = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        img = cv2.erode(img, erodeEl)
        img = cv2.dilate(img, dilateEl)
        img = cv2.morphologyEx(img, cv2.MORPH_OPEN, openEl)

        return img

    def findContourAndBound(self, img, bounded=True, minArea=0.0):
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_NONE)
        if bounded:
            contours = filter(lambda c: cv2.contourArea(c) > minArea,
                              contours)
        return contours

    def enhance(self, img):
        enhancedImg = cv2.GaussianBlur(img, ksize=(0, 0), sigmaX=10)
        enhancedImg = cv2.addWeighted(img, 2.5, enhancedImg, -1.5, 0)
        return enhancedImg

    def makeLineEquation(self, p1, p2):
        """ Return a higher order function that represent
            a line made by 2 points """
        def equation(pt):
            x = pt[0]
            y = pt[1]
            return (y-p1[1])-((p2[1]-p1[1])/(p2[0]-p1[0]))*(x-p1[0])

        return equation

    def contain(self, rect, point):
        """ Check if a rectangle (4 points) contain a point """
        for i in range(4):
            lineEq = self.makeLineEquation(rect[i], rect[(i+1)%4])
            p1 = lineEq(rect[(i+2)%4])
            p2 = lineEq(point)
            if (p1 < 0 and p2 >= 0) or (p1 > 0 and p2 <= 0):
                return False

        return True

    def match(self, centroids, rects):
        """ Match each centroid in centroids (x, y) to a rect in rects
            (4 points) if the centroid is inside the rectangle
            return list of dicts->{centroid, rect} """
        ret = list()
        for centroid in centroids:
            for rect in rects:
                if self.contain(rect, centroid):
                    ret.append({'centroid':centroid, 'rect': rect})

        return ret

    def gotFrame(self, img):
        """ Main processing function, should return (retData, outputImg) """
        foundRects = list()
        centroids = list()
        outImg = None
        matches = list()
        retData = {'foundRects': foundRects, 'centroids': centroids,
                   'matches': matches}

        img = cv2.resize(img, (self.screen['width'], self.screen['height']))
        img = self.enhance(img)
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        binImg = cv2.inRange(hsvImg, self.hsvLoThresh1, self.hsvHiThresh1)
        binImg |= cv2.inRange(hsvImg, self.hsvLoThresh2, self.hsvHiThresh2)

        binImg = self.morphology(binImg)

        if self.debugMode:
            outImg = cv2.cvtColor(binImg.copy(), cv2.COLOR_GRAY2BGR)

        scratchImg = binImg.copy()
        contours = self.findContourAndBound(scratchImg,
                                            bounded=True,
                                            minArea=self.minContourArea)
        if not contours or len(contours) < 1: return retData, outImg
        sorted(contours, key=cv2.contourArea, reverse=True)

        for contour in contours:
            moment = cv2.moments(contour, False)
            centroids.append((moment['m10']/moment['m00'],
                              moment['m01']/moment['m00']))

        if self.debugMode:
            for centroid in centroids:
                cv2.circle(outImg, (int(centroid[0]), int(centroid[1])), 5,
                           (0, 0, 255))

        grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mean = cv2.mean(grayImg)[0]
        lowest = cv2.minMaxLoc(grayImg)[0]
        thVal = min((lowest + mean)/3.99, self.upperThresh)
        grayImg = cv2.threshold(grayImg, thVal, 255, cv2.THRESH_BINARY_INV)[1];

        contours = self.findContourAndBound(grayImg, minArea=self.areaThresh)
        sorted(contours, key=cv2.contourArea, reverse=True)
        contourRects = [cv2.cv.BoxPoints(cv2.minAreaRect(contour))
                        for contour in contours]

        if self.debugMode:
            for rect in contourRects:
                Vision.drawRect(outImg, rect)

        matches = self.match(centroids, contourRects)
        retData['matches'] = matches

        return retData, outImg

def main():
    cv2.namedWindow("output")
    img = cv2.imread("bins/bins_layout.jpg")
    from comms import Comms
    visionFilter = BinsVision(comms=Comms())
    _, outImg = visionFilter.gotFrame(img)
    if outImg is not None: cv2.imshow("output", outImg)
    cv2.waitKey()
