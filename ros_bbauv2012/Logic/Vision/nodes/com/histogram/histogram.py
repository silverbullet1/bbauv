'''
Created on Apr 15, 2013

@author: gohew
'''
import cv2 as cv2
import cv2.cv as cv
import numpy as np

class Hist_constants():
    SINGLE_CHANNEL = 0
    TRIPLE_CHANNEL = 1
    DUAL_CHANNEL_MODE = 2
    
class bbHistogram():
    '''
    classdocs
    '''
    windowName = ["Hue histogram", "Sat histogram","Value histogram"]
    windowColor = [(255,0,0),(0,255,0),(0,0,255)]
    params = { 'satLow': 0, 'satHigh': 255, 'hueLow': 0, 'hueHigh':255,'valLow':0,'valHigh':255,'grayLow': 0, 'grayHigh': 255}
    thresColor = (100,50,200)
    type = None
    def __init__(self,type=Hist_constants.TRIPLE_CHANNEL):
        '''
        Constructor
        '''
        self.type = type
        
        if(type == Hist_constants.TRIPLE_CHANNEL):
            cv2.namedWindow("histogram settings",cv2.CV_WINDOW_AUTOSIZE)
            cv2.createTrackbar("Hue Low:", "histogram settings", self.params['hueLow'], 255, self.paramSetter('hueLow'));
            cv2.createTrackbar("Hue High:", "histogram settings", self.params['hueHigh'], 255, self.paramSetter('hueHigh'));
            cv2.createTrackbar("Saturation Low:", "histogram settings", self.params['satLow'], 255, self.paramSetter('satLow'));
            cv2.createTrackbar("Saturation High:", "histogram settings", self.params['satHigh'], 255, self.paramSetter('satHigh'));
            cv2.createTrackbar("Value Low:", "histogram settings", self.params['valLow'], 255, self.paramSetter('valLow'));
            cv2.createTrackbar("Value High:", "histogram settings", self.params['valHigh'], 255, self.paramSetter('valHigh'));
            cv2.namedWindow("3 Channel Histogram",cv2.CV_WINDOW_AUTOSIZE)
            cv2.moveWindow("3 Channel Histogram",0,30)
            cv2.moveWindow("histogram settings",256,30)
        elif(type == Hist_constants.SINGLE_CHANNEL):
            cv2.namedWindow("1 Channel Histogram",cv2.CV_WINDOW_AUTOSIZE)
            cv2.createTrackbar("Gray Low:", "1 Channel Histogram", self.params['grayLow'], 255, self.paramSetter('grayLow'));
            cv2.createTrackbar("Gray High:", "1 Channel Histogram", self.params['grayHigh'], 255, self.paramSetter('grayHigh'));
            cv2.moveWindow("1 Channel Histogram",0,30)
        elif(type == Hist_constants.DUAL_CHANNEL_MODE):
            cv2.namedWindow("1 Channel Histogram",cv2.CV_WINDOW_AUTOSIZE)
            cv2.createTrackbar("Hue Low:", "1 Channel Histogram", self.params['hueLow'], 255, self.paramSetter('hueLow'));
            cv2.createTrackbar("Hue High:", "1 Channel Histogram", self.params['hueHigh'], 255, self.paramSetter('hueHigh'));
            cv2.createTrackbar("Saturation Low:", "1 Channel Histogram", self.params['satLow'], 255, self.paramSetter('satLow'));
            cv2.createTrackbar("Saturation High:", "1 Channel Histogram", self.params['satHigh'], 255, self.paramSetter('satHigh'));
            cv2.createTrackbar("Value Low:", "1 Channel Histogram", self.params['valLow'], 255, self.paramSetter('valLow'));
            cv2.createTrackbar("Value High:", "1 Channel Histogram", self.params['valHigh'], 255, self.paramSetter('valHigh'));
            cv2.createTrackbar("Gray Low:", "1 Channel Histogram", self.params['grayLow'], 255, self.paramSetter('grayLow'));
            cv2.createTrackbar("Gray High:", "1 Channel Histogram", self.params['grayHigh'], 255, self.paramSetter('grayHigh'));
            cv2.namedWindow("3 Channel Histogram",cv2.CV_WINDOW_AUTOSIZE)
            cv2.moveWindow("3 Channel Histogram",0,30)
            cv2.moveWindow("1 Channel Histogram",256,30)
        else:
            print "Please specify a correct type. Specify either TRIPLE_CHANNEL, SINGLE_CHANNEL or DUAL_CHANNEL_MODE"
   
    ''' 
    Utility Methods
    '''
            
   
    def ChannelEqualize(self,image, triple):
    # image: Triple or Single channel image
    # triple: Boolean value to indicate 3 channel or 1 channel. True for triple channel
        if triple == True:
            equal_array = cv2.split(image)
            for i in range(0,3):
                equal_array[i] = cv2.equalizeHist(equal_array[i])
                equal_image = cv2.merge(equal_array)
            return equal_image
        else:
            return cv2.equalizeHist(image)
    
    def setTrackBarPosition(self,name,val):
        if self.type == Hist_constants.TRIPLE_CHANNEL:
            cv2.setTrackbarPos(str(name), "histogram settings", val)
        else:
            cv2.setTrackbarPos(str(name), "1 Channel Histogram", val)
        
    def paramSetter(self,key):
        def setter(val):
            self.params[key] = val
        return setter
    def setParams(self,parameters):
        self.params = parameters
    def getParams(self):
        return self.params
    #Compute the Histogram for three channels. Takes in a three channel image array.
    def getTripleHist(self,image):
        hist_bins = 256
        window_height = 200
        ranges = [0,255]
        i = 0
        if(self.type == Hist_constants.TRIPLE_CHANNEL or self.type == Hist_constants.DUAL_CHANNEL_MODE):
            imgArray = cv2.split(image)
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
                    cv2.line(histImg, (h,window_height*(i+1)), (h, window_height*(i+1)-binVal), self.windowColor[i],thickness=1)
                i = i + 1
            cv2.imshow("3 Channel Histogram",histImg)
        else:
            print "Switch mode to TRIPLE_CHANNEL or DUAL_CHANNEL_MODE!"   
           
    def getSingleHist(self,image):
        hist_bins = 256
        window_height = 200
        ranges = [0,255]
        if(self.type == Hist_constants.SINGLE_CHANNEL or self.type == Hist_constants.DUAL_CHANNEL_MODE):
                histImg = np.zeros((200,256,3),dtype=np.uint8)
                cv2.line(histImg,(self.params['grayLow'],window_height*(1)),(self.params['grayLow'],window_height*(1) -window_height),self.thresColor)
                cv2.line(histImg,(self.params['grayHigh'],window_height*(1)),(self.params['grayHigh'],window_height*(1) -window_height),self.thresColor)
                hist, _ = np.histogram(image, hist_bins, (0, 255))
                cv2.normalize(hist,hist,0,200,cv2.NORM_MINMAX)
                for h in range(hist_bins):
                        binVal = int(hist[h])
                        cv2.line(histImg, (h,window_height*(1)), (h, window_height*(1)-binVal), (255,255,255),thickness=1)
                cv2.imshow("1 Channel Histogram",histImg)
        else:
            print "Switch mode to SINGLE_CHANNEL or DUAL_CHANNEL_MODE!"       
    
    #Draw Histogram on a single Image Channel (Jon's Method)
    def get_hist_img(self,cv_img):
        hist_bins = 256
        hist_ranges = [(0,255)]
    
        hist, _ = np.histogram(cv_img, hist_bins, (0, 255))
        maxVal = np.max(hist)
    
        #histImg = np.array( [255] * (hist_bins * hist_bins), dtype=np.uint8 ).reshape([hist_bins, hist_bins])
        histImg = np.zeros((200,256),dtype=np.uint8)
        hpt = int(0.5 * hist_bins)
    
        for h in range(hist_bins):
            binVal = float(hist[h])
            intensity = int(binVal * hpt / maxVal)
            cv2.line(histImg, (h, hist_bins), (h, hist_bins-intensity), (100,100,100))
    
        return histImg

        