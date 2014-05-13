import cv2
import math

class Vision():
    SHAPE_TRI = 0
    SHAPE_RECT = 1
    SHAPE_SQU = 2
    SHAPE_PENTA = 3
    SHAPE_HEXA = 4
    SHAPE_NONCONVEX = 5
    SHAPE_CIR = 6

    _cosBounds = {'rect': (-0.1, 0.3), 'penta': (-0.34, -0.27),
                  'hexa': (-0.55, -0.45)}

    @staticmethod
    def shapeEnum2Str(shapeEnum):
        if shapeEnum is None:
            return 'UNKNOWN'

        if shapeEnum == Vision.SHAPE_TRI:
            return 'TRI'
        if shapeEnum == Vision.SHAPE_RECT:
            return 'RECT'
        if shapeEnum == Vision.SHAPE_SQU:
            return 'SQU'
        if shapeEnum == Vision.SHAPE_PENTA:
            return 'PENTA'
        if shapeEnum == Vision.SHAPE_HEXA:
            return 'HEXA'
        if shapeEnum == Vision.SHAPE_CIR:
            return 'CIR'
        if shapeEnum == Vision.SHAPE_NONCONVEX:
            return 'CONCAVE'

        return 'UNKNOWN'

    @staticmethod
    def angle(pt1, pt2, pt0):
        """ Find angle around p0 made by p1 and p2
            p1, p2, p0 are 2D points: (x-coord, y-coord) """
        dx1 = pt1[0] - pt0[0]
        dy1 = pt1[1] - pt0[1]
        dx2 = pt2[0] - pt0[0]
        dy2 = pt2[1] - pt0[1]
        return float(dx1*dx2 + dy1*dy2)/math.sqrt((dx1*dx1 + dy1*dy1) *
                                                  (dx2*dx2 + dy2*dy2) + 1e-10)

    @staticmethod
    def detectShape(contour, multiplier=0.02):
        #if not cv2.isContourConvex(contour):
        #    return Vision.SHAPE_NONCONVEX
        approx = cv2.approxPolyDP(contour,
                                  cv2.arcLength(contour, True)*multiplier,
                                  True)
        if len(approx) == 3:
            return Vision.SHAPE_TRI
        elif len(approx) >= 4 and len(approx) <= 6:
            vtc = len(approx)

            # Find the cosine for angles of the polygon
            cos = list()
            for i in range(2, vtc+1):
                cos.append(Vision.angle(approx[i%vtc][0],
                                        approx[i-2][0],
                                        approx[i-1][0])) 
            # Sort the angles cosine from smallest to largest
            sorted(cos)

            # Get the smallest and largest angle cosine
            mincos = cos[0]
            maxcos = cos[-1]
            
            if vtc == 4 and \
               mincos >= Vision._cosBounds['rect'][0] and \
               maxcos <= Vision._cosBounds['rect'][1]:
                r = cv2.boundingRect(contour)
                ratio = abs(1 - float(r[2])/r[3])
                return Vision.SHAPE_SQU if ratio <= 0.02 else Vision.SHAPE_RECT
            elif vtc == 5 and \
                    mincos >= Vision._cosBounds['penta'][0] and \
                    maxcos <= Vision._cosBounds['penta'][1]:
                return Vision.SHAPE_PENTA
            elif vtc == 6 and \
               mincos >= Vision._cosBounds['hexa'][0] and \
               maxcos <= Vision._cosBounds['hexa'][1]:
                return Vision.SHAPE_HEXA
        else:
            area = cv2.contourArea(contour)
            r = cv2.boundingRect(contour)
            radius = r[2]/2.0

            if abs(1 - float(r[2]) / r[3]) <= 0.2 and \
               abs(1 - (area / (math.pi * radius**2))) <= 0.2:
                return Vision.SHAPE_CIR

        return None

def main():
    img = cv2.imread("bot_common/shapes.png")
    grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edgeImg = cv2.Canny(grayImg, 0, 50, 5)
    contours, _ = cv2.findContours(edgeImg, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_NONE)
    for contour in contours:
        shape = Vision.detectShape(contour)
        pos = (cv2.minAreaRect(contour)[0])
        pos = (int(pos[0]), int(pos[1]))
        cv2.putText(img, Vision.shapeEnum2Str(shape), pos,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    cv2.imshow("display", img)
    cv2.waitKey()
