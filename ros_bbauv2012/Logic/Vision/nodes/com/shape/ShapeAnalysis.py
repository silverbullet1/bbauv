'''
Created on May 5, 2013

@author: gohew
'''
import cv2 as cv2
import cv2.cv as cv
import numpy as np
import os
import sys

class ShapeAnalysis():
    '''
    classdocs
    '''
    knn = cv2.KNearest()
    
    trainingContour = list()
    def __init__(self):
        '''
        Constructor
        '''
        os.chdir("/home/gohew/fuerte_workspace/bbauv/ros_bbauv2012/Logic/Vision/nodes/assets/")
        
        cv2.namedWindow("SURF",cv2.CV_WINDOW_AUTOSIZE)
        #cv2.namedWindow("training data")
        cv2.moveWindow("SURF",512,350)
        #cv2.namedWindow("Shape Target Analysis",cv2.CV_WINDOW_AUTOSIZE)
        #cv2.moveWindow("Shape Target Analysis", 512, 350)
        self.initSurf()
    
    def initSurf(self):
        for i in range(2,3):
            trgData =cv2.imread('pic' + str(i) + '.png')
            trgData = cv2.split(trgData)
            print trgData
            #Compute SURF on Red Channel
            hkeypoints,hdescriptors = self.computeSurf(trgData[2])
            # extract vectors of size 64 from raw descriptors numpy arrays
            rowsize = len(hdescriptors) / len(hkeypoints)
            if rowsize > 1:
                hrows = np.array(hdescriptors, dtype = np.float32).reshape((-1, rowsize))
                #print hrows.shape, nrows.shape
            else:
                hrows = np.array(hdescriptors, dtype = np.float32)
                rowsize = len(hrows[0])
            # kNN training - learn mapping from hrow to hkeypoints index
            samples = hrows
            responses = np.arange(len(hkeypoints), dtype = np.float32)
            print len(samples), len(responses)
            self.knn.train(samples,responses)
            
    def predictSurf(self,image):
        hkeypoints,hdescriptors = self.computeSurf(image)
        # extract vectors of size 64 from raw descriptors numpy arrays
        rowsize = len(hdescriptors) / len(hkeypoints)
        if rowsize > 1:
            hrows = np.array(hdescriptors, dtype = np.float32).reshape((-1, rowsize))
            #print hrows.shape, nrows.shape
        else:
            hrows = np.array(hdescriptors, dtype = np.float32)
            rowsize = len(hrows[0])
        # kNN training - learn mapping from hrow to hkeypoints index
        samples = hrows        
        # retrieve index and value through enumeration
        for i, descriptor in enumerate(hrows):
            descriptor = np.array(descriptor, dtype = np.float32).reshape((1, rowsize))
            #print i, descriptor.shape, samples[0].shape
            retval, results, neigh_resp, dists = self.knn.find_nearest(descriptor, 1)
            res, dist =  int(results[0][0]), dists[0][0]
            #print res, dist
        
            if dist < 0.1:
                # draw matched keypoints in red color
                color = (0, 0, 255)
            else:
                # draw unmatched in blue color
                color = (255, 0, 0)
            # draw matched key points on haystack image
            #print res
            #print len(hkeypoints)
            #print hkeypoints[res]
            x,y = hkeypoints[i].pt
            center = (int(x),int(y))
            cv2.circle(image,center,2,color,-1)
            # draw matched key points on needle image
            #x,y = nkeypoints[i].pt
            #center = (int(x),int(y))
            #cv2.circle(opencv_needle,center,2,color,-1)
            cv2.imshow("SURF",image)
            
    def initMatchShape(self):
        trgData = np.array(cv2.imread("assets/sword.png"),dtype=np.uint8)
        trgData = cv2.cvtColor(trgData,cv2.COLOR_BGR2GRAY)
        self.trainingContour.append(self.computeMoments(trgData))
        trgData = np.array(cv2.imread("assets/shield.png"),dtype=np.uint8)
        trgData = cv2.cvtColor(trgData,cv2.COLOR_BGR2GRAY)
        self.trainingContour.append(self.computeMoments(trgData))
        
    def computeMoments(self,image):
        retval,thresImg = cv2.threshold(image, 230 , 255, cv2.THRESH_BINARY_INV)
        trgContours,hierarchy = cv2.findContours(thresImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        #cv2.imshow("training data",np.array(thresImg,dtype=np.uint8))
        print "shape" + str(trgContours[0].shape)
        return trgContours
    
    '''
    Method to perform SURF analysis with kNN evaluation
    '''
    def computeSurf(self,image):
        imageArray = np.array(image,dtype=np.uint8)
        storage = None
        keypoints = None
        # build feature detector and descriptor extractor
        hessian_threshold = 85
        detector = cv2.SURF(hessian_threshold)
        (keypoints, descriptors) = detector.detect(image, None, useProvidedKeypoints = False)
        
        return keypoints,descriptors
        '''
        for i in keypoints:
            x,y = i.pt
            cv2.circle(imageArray,(int(x),int(y)), 2, (0,0,255), thickness=1)
                        
        #keypoints,descriptor = cv.ExtractSURF(image, 0, storage, params)
        cv2.imshow("SURF", imageArray)
        '''
        
    '''
    Function to do shape analysis with the matchShapes Algorithm
    http://opencv.willowgarage.com/documentation/cpp/imgproc_structural_analysis_and_shape_descriptors.html#matchShapes
    
    '''
    def findShapeTarget(self,image):
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        #cv_single = cv2.morphologyEx(cv_single, cv2.MORPH_OPEN,kernel)
        image = cv2.morphologyEx(image, cv2.MORPH_CLOSE,kernel_close,iterations=2)
        contoursTarget,hierarchyTarget = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        targetImg = np.zeros((240,320,3),dtype=np.uint8)
        minContour = 5000
        minContourIdx = 0
        contourListIdx = list()
        errorList = list()
        for i in range(0,len(contoursTarget)):
            moments =cv2.moments(contoursTarget[i],binaryImage=False)
            if moments['m00'] > 200 and moments['m00'] < 7000:
                humoments = cv2.HuMoments(moments)
                cv2.drawContours(targetImg, contoursTarget, i, (100,255,100),thickness= 1) 
                '''y
                if abs(humoments[1][0] - 0.8) < 0.005:
                    colorText = (0,0,255)
                    imgTxt = "sword:" + "u1:" + str(np.round(humoments[1][0],3))
                elif abs(humoments[1][0] - 0.020) < 0.02:
                    colorText = (0,0,255)
                    imgTxt = "window:" + "u1:" + str(np.round(humoments[1][0],3))
                else:
                    imgTxt = "u1:" + str(np.round(humoments[1][0],3))
                    colorText = (100,100,100)
                '''
                error = cv2.matchShapes(self.trainingContour[1][0],contoursTarget[i], cv2.cv.CV_CONTOURS_MATCH_I1,0)
                errorList.append(error)
                contourListIdx.append(i)
                if(minContour > error):
                    minContour = error
                    minContourIdx = i
                print minContour
        for i in range(0,len(errorList)):
            if(minContourIdx == contourListIdx[i]):
                colorText = (0,0,255)
                imgTxt = "shield:" + str(np.round(errorList[i],3))
            else:
                colorText = (100,100,100)
                imgTxt = str(np.round(errorList[i],3))
            cv2.putText(targetImg,imgTxt, (contoursTarget[contourListIdx[i]][0][0][0],contoursTarget[contourListIdx[i]][0][0][1]), cv2.FONT_HERSHEY_PLAIN, 0.8, colorText)
        cv2.imshow("Shape Target Analysis", targetImg)
        return targetImg
        
        