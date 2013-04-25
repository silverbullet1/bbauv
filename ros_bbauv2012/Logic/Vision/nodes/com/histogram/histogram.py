'''
Created on Apr 15, 2013

@author: gohew
'''
import cv2 as cv2
import cv2.cv as cv
import numpy as np

class bbHistogram():
    '''
    classdocs
    '''
    windowName = ["Hue histogram", "Sat histogram","Value histogram"]
    windowColor = [(255,0,0),(0,255,0),(0,0,255)]
    params = { 'satLow': 0, 'satHigh': 255, 'hueLow': 48, 'hueHigh':86,'valLow':0,'valHigh':255}
    thresColor = (100,50,200)
    
    def __init__(self):
        '''
        Constructor
        '''
        cv2.namedWindow("histogram settings",cv2.CV_WINDOW_AUTOSIZE)
        cv2.createTrackbar("Hue Low:", "histogram settings", self.params['hueLow'], 255, self.paramSetter('hueLow'));
        cv2.createTrackbar("Hue High:", "histogram settings", self.params['hueHigh'], 255, self.paramSetter('hueHigh'));
        cv2.createTrackbar("Saturation Low:", "histogram settings", self.params['satLow'], 255, self.paramSetter('satLow'));
        cv2.createTrackbar("Saturation High:", "histogram settings", self.params['satHigh'], 255, self.paramSetter('satHigh'));
        cv2.createTrackbar("Value Low:", "histogram settings", self.params['valLow'], 255, self.paramSetter('valLow'));
        cv2.createTrackbar("Value High:", "histogram settings", self.params['valHigh'], 255, self.paramSetter('valHigh'));
        cv2.namedWindow("HSV histogram",cv2.CV_WINDOW_AUTOSIZE)
        cv2.moveWindow("HSV histogram",0,30)
        cv2.moveWindow("histogram settings",256,30)
        
    def paramSetter(self,key):
        def setter(val):
            self.params[key] = val
        return setter
    def setParams(self,parameters):
        self.params = parameters
    def getParams(self):
        return self.params
    #Compute the Histogram for three channels. Takes in a three channel image array.
    def calcHist(self,image):
        #self.fig.clear()
        hist_bins = 256
        window_height = 200
        ranges = [0,255]
        channel = [1]
        aRange = [ranges,ranges,ranges]
        imgArray = cv2.split(image)
        i = 0
        histImg = np.zeros((600,256,3),dtype=np.uint8)
        cv2.line(histImg,(self.params['hueLow'],window_height*(1)),(self.params['hueLow'],window_height*(1) -window_height),self.thresColor)
        cv2.line(histImg,(self.params['hueHigh'],window_height*(1)),(self.params['hueHigh'],window_height*(1) -window_height),self.thresColor)
        cv2.line(histImg,(self.params['satLow'],window_height*(2)),(self.params['satLow'],window_height*(2) -window_height),self.thresColor)
        cv2.line(histImg,(self.params['satHigh'],window_height*(2)),(self.params['satHigh'],window_height*(2) -window_height),self.thresColor)
        cv2.line(histImg,(self.params['valLow'],window_height*(3)),(self.params['valLow'],window_height*(3) -window_height),self.thresColor)
        cv2.line(histImg,(self.params['valHigh'],window_height*(3)),(self.params['valHigh'],window_height*(3) -window_height),self.thresColor)
        
        for channelImg in imgArray:
            #hist = cv2.calcHist(channelImg, channel, None, [256], ranges)
            hist, _ = np.histogram(channelImg, hist_bins, (0, 255))
            cv2.normalize(hist,hist,0,200,cv2.NORM_MINMAX)
            for h in range(hist_bins):
                binVal = int(hist[h])
                cv2.line(histImg, (h,window_height*(i+1)), (h, window_height*(i+1)-binVal), self.windowColor[i],thickness=2)
            i = i + 1
        cv2.imshow("HSV histogram",histImg)
            
            #cv2.imshow("Sat histogram",tempHist)
                
    #Draw Histogram on a single Image Channel (Jon's Method)
    def get_hist_img(self,cv_img):
        hist_bins = 256
        hist_ranges = [(0,255)]
    
        hist, _ = np.histogram(cv_img, hist_bins, (0, 255))
        maxVal = np.max(hist)
    
        histImg = np.array( [255] * (hist_bins * hist_bins), dtype=np.uint8 ).reshape([hist_bins, hist_bins])
        hpt = int(0.5 * hist_bins)
    
        for h in range(hist_bins):
            binVal = float(hist[h])
            intensity = int(binVal * hpt / maxVal)
            cv2.line(histImg, (h, hist_bins), (h, hist_bins-intensity), (100,100,100))
    
        return histImg

        