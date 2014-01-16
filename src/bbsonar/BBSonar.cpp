/*
 * BBSonar.cpp
 *
 *  Created on: Dec 14, 2013
 *      Author: freax
 */

#include "BBSonar.h"

BBSonar::BBSonar() : son(NULL), fson(NULL), head(NULL), ping(NULL), rangeData(NULL),
	magImg(NULL), colorImg(NULL), colorMap(NULL), grayImg(NULL), grayImg8b(NULL), grayImg16b(NULL)
{
	retVal = startRange = stopRange = fluidType = soundSpeed = analogGain = tvGain =
	pingCount = imgWidth = imgHeight = imgWidthStep = imgFilterFlags= 0;
}

BBSonar::~BBSonar() {
	if (NULL != son) BVTSonar_Destroy(son);
	if (NULL != ping) BVTPing_Destroy(ping);
	if (NULL != grayImg8b) cvReleaseImage(& grayImg8b);
	if (NULL != grayImg16b) cvReleaseImage(& grayImg16b);
}

int BBSonar::initSonar() {
	son = BVTSonar_Create();
	if (NULL == son) {
		cout << "son null, exiting now" << endl;
		return retVal;
	}

#ifdef ETH_SONAR
	retVal = BVTSonar_Open(son, "NET", "192.168.1.45");
#else
	retVal = BVTSonar_Open(son, "FILE", "sonarLog.son");
#endif
	cout << "son initialization: " << BVTError_GetString(retVal) << endl;

	if((retVal = BVTSonar_GetHead(son, HEAD_NUM, &head)) != 0) {
		cout << "error retrieving head, exiting now: " << BVTError_GetString(retVal) << endl;
		return retVal;
	}
	return 0;
}

void BBSonar::getSetHeadParams() {
	//gets
	startRange = BVTHead_GetStartRange(head);
	stopRange = BVTHead_GetStopRange(head);
	fluidType = BVTHead_GetFluidType(head);
	soundSpeed = BVTHead_GetSoundSpeed(head);
	analogGain = BVTHead_GetGainAdjustment(head);
	tvGain = BVTHead_GetTVGSlope(head);
	pingCount = BVTHead_GetPingCount(head);
	imgFilterFlags = BVTHead_GetImageFilterFlags(head);

	cout << "(Start, Stop) range:\t" << BVTHead_GetStartRange(head) << "\t" << BVTHead_GetStopRange(head) << endl;
	cout << "(Min, Max) range:\t" << BVTHead_GetMinimumRange(head) << "\t" << BVTHead_GetMaximumRange(head) << endl;
	cout << "Fluid type: " << BVTHead_GetFluidType(head) << "\t" << endl;

	//sets
//	not working
//	if((retVal = BVTHead_SetRangeResolution(head, RES_IN_METRES)) != 0)
//		cout << "error setting range resolution" << endl;
//
	if((retVal = BVTHead_SetImageRes(head, RES_TYPE)) != 0)
		cout << "error setting image resolution" << endl;

	if((retVal = BVTHead_SetImageType(head, IMAGE_TYPE)) != 0)
		cout << "error setting image type" << endl;

	if((retVal = BVTHead_SetImageFilterFlags(head, imgFilterFlags)) != 0)
		cout << "error setting image filter flags" << endl;

	if((retVal = BVTHead_SetGainAdjustment(head, analogGain)) != 0)
		cout << "error setting analog gain" << endl;

	if((retVal = BVTHead_SetTVGSlope(head, tvGain)) != 0)
		cout << "error setting TV Gain" << endl;

}

int BBSonar::retrievePing() {
	if((retVal = BVTHead_GetPing(head, PING_NUM,  &ping)) != 0) {
		cout << "error retrieving ping: "  << BVTError_GetString(retVal) << endl;
		return retVal;
	}
	if((retVal = BVTPing_GetImage(ping, &magImg)) != 0) {
		cout << "error retrieving ping: " << BVTError_GetString(retVal) << endl;
		return retVal;
	}
	this->imgWidth = BVTMagImage_GetWidth(magImg);
//	imgWidth in multiple of DWORD(=4) size
	this->imgWidth += (this->imgWidthStep % 4) ;
	this->imgHeight = BVTMagImage_GetHeight(magImg);
	cout << "Image (imgHeight, imgWidth) =\t" << imgHeight << "\t" << imgWidth << endl;

//	if (NULL != ping) BVTPing_Destroy(ping);
	return 0;
}

int BBSonar::saveRangeBearingData() {
	float range, bearing;

	if((retVal = BVTPing_GetImage(ping, &magImg)) != 0) {
		cout << "error retrieving ping: " << BVTError_GetString(retVal) << endl;
		return retVal;
	}
	this->imgHeight = BVTMagImage_GetHeight(magImg);
	this->imgWidth = BVTMagImage_GetWidth(magImg);
	cout << "Image resolution: " << this->imgHeight << " X " << this->imgWidth << " pixels" << endl;

	rangeStream.open(RANGE_DATA_PATH.c_str());
	bearingStream.open(BEARING_DATA_PATH.c_str());

	rangeStream.precision(5);
	bearingStream.precision(5);

	for(int r=0; r<imgHeight; r++) {
		for(int c=0; c<imgWidth; c++) {
			range=BVTMagImage_GetPixelRange(magImg, r, c);
			bearing=BVTMagImage_GetPixelRelativeBearing(magImg, r, c);
			rangeStream << range << " ";
			bearingStream << bearing << " ";
		}
		rangeStream << endl;
		bearingStream << endl;
	}

	rangeStream.close();
	bearingStream.close();

	if (NULL != magImg) BVTMagImage_Destroy(magImg);
	if (NULL != colorMap) BVTColorMapper_Destroy(colorMap);
	if (NULL != colorImg) BVTColorImage_Destroy(colorImg);

	return 0;
}

int BBSonar::processPing() {
//	IplImage* grayImg = NULL;
	if((retVal = BVTPing_GetImage(ping, &magImg)) != 0) {
		cout << "error retrieving ping: " << BVTError_GetString(retVal) << endl;
		return retVal;
	}
	if((colorMap = BVTColorMapper_Create()) == NULL) {
		cout << "error creating color mapper" << endl;
		return -1;
	}
	if((retVal = BVTColorMapper_Load(colorMap, COLOR_MAPPER_PATH.c_str())) != 0) {
		cout << "error retrieving color map: " << BVTError_GetString(retVal) << endl;
		return retVal;
	}
	if((retVal = BVTColorMapper_MapImage(colorMap, magImg, &colorImg)) != 0) {
		cout << "error mapping to color image: " << BVTError_GetString(retVal) << endl;
		return retVal;
	}

//	obtaining a 16-bit grayscale image whose pixel values will range from 0-65535
	grayImg = cvCreateImageHeader(cvSize(imgWidth, imgHeight), IPL_DEPTH_16U, 1);
	cvSetImageData(grayImg, BVTMagImage_GetBits(magImg), imgWidth * 2);
	imgWidthStep = grayImg->widthStep;

//	save the sonar images as well as the grayscale intensities in respective files
#ifndef SAVE_SONAR_IMAGES
	if((retVal = BVTMagImage_SavePGM(magImg, GRAYSCALE_IMAGE_FILE.c_str())) != 0) {
		cout << "error saving grayscale image file: " << BVTError_GetString(retVal) << endl;
		return retVal;
	}
	if((retVal = BVTColorImage_SavePPM(colorImg, COLOR_IMAGE_FILE.c_str())) != 0) {
		cout << "error saving color image file: " << BVTError_GetString(retVal) << endl;
		return retVal;
	}

	retVal = writeIntensities(grayImg);
	if (retVal != 0) cout << "error saving intensities" << endl;

	cvNamedWindow("Gray image", 0);
	cvShowImage("Gray image", grayImg);
	cvWaitKey(0);
#else //read the saved intensities file and create the grayscale image (8-bit) out of it for processing
	retVal = readIntensities();
	if (retVal != 0) cout << "error reading intensities" << endl;

	if (processImage() != 0) cout << "error processing grayscale image read from intensities" << endl;
#endif

	cvReleaseImageHeader(&grayImg);
	cvDestroyAllWindows();
	if (NULL != magImg) BVTMagImage_Destroy(magImg);
	if (NULL != colorMap) BVTColorMapper_Destroy(colorMap);
	if (NULL != colorImg) BVTColorImage_Destroy(colorImg);
	return 0;
}

int BBSonar::processImage() {
//	all the images are 8-bit single channeled derived from grayImg8b
	IplImage *gImg=NULL, *sImg=NULL, *eImg=NULL, *tImg=NULL,
			*mImg=NULL, *lImg=NULL, *oImg=NULL ;
	double threshVal = 0;
	int elementShape = CV_SHAPE_ELLIPSE;
	IplConvKernel* structElement = NULL;
	CvSeq* contourItx = NULL;
	CvMemStorage* memStorage = NULL;
	CvScalar sColor;
	int numObjects = 0, labelColor ;

	gImg = cvCreateImage(cvSize(imgWidth, imgHeight), IPL_DEPTH_8U, 1);
	sImg = cvCreateImage(cvSize(imgWidth, imgHeight), IPL_DEPTH_8U, 1);
	eImg = cvCreateImage(cvSize(imgWidth, imgHeight), IPL_DEPTH_8U, 1);
	tImg = cvCreateImage(cvSize(imgWidth, imgHeight), IPL_DEPTH_8U, 1);
	mImg = cvCreateImage(cvSize(imgWidth, imgHeight), IPL_DEPTH_8U, 1);
	lImg = cvCreateImage(cvSize(imgWidth, imgHeight), IPL_DEPTH_8U, 1);

	cvCopy(grayImg8b, gImg);

//	median filtering: aperture width = 3
	cvSmooth(gImg, sImg, CV_MEDIAN, 3, 0, 0, 0);

//	thresholding using the global threshold alg
	threshVal = getGlobalThreshold(gImg);
	cout << "global threshold value: " << threshVal << endl ;
	cvThreshold(sImg, tImg, threshVal, 255, CV_THRESH_BINARY) ;

//	applying morphology (dilation) to the thresholded image: kernel (3, 3)
	structElement = cvCreateStructuringElementEx(3, 3, 0, 0, elementShape, 0);
	cvDilate(tImg, mImg, structElement, 1);
	cvNamedWindow("Dilated image", 0);
	cvShowImage("Dilated image", mImg);

//	canny-filter for edge detection: threshold range (1.0, 3.0), kernel (3X3)
	cvCanny(mImg, eImg, 1.0, 3.0, 3);

//	creating a labeled image
	cvZero(lImg);
	memStorage = cvCreateMemStorage(0);
	numObjects = cvFindContours(mImg, memStorage, &contourItx, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	labelColor = HOTSPOT_THRESH;
	for (; contourItx!=0; contourItx=contourItx->h_next) {
		sColor = cvScalar(labelColor);
		cvDrawContours(lImg, contourItx, sColor, sColor, -1, 1, 8);
		labelColor++;
	}
//	check : lImg on window
	cout << "numObjects detected: " << numObjects << endl;

// 	Obtain labeled objects sizes by finding bounding rectangles
	CvPoint pt1, pt2;
	for (int itX = HOTSPOT_THRESH; itX < HOTSPOT_THRESH + numObjects; ++itX) {
		oImg = cvCreateImage(cvSize(imgWidth, imgHeight), IPL_DEPTH_8U, 1);
		cvZero(oImg);
		cvCmpS(lImg, double(itX), oImg, CV_CMP_EQ);
		CvRect objRect = cvBoundingRect(oImg, 1);
		cout << "Object " << itX - HOTSPOT_THRESH + 1 << ": " << "[ col=" << objRect.x << ", " << "row=" << objRect.y << ", "
				<< "w=" << objRect.width << ", " << "h=" << objRect.height << " ]" << endl;

// 		Plot bounding rectangle on labeled object image
		pt1.x = objRect.x;
		pt1.y = objRect.y;
		pt2.x = objRect.x + objRect.width;
		pt2.y = objRect.y + objRect.height;
		if (objRect.width ==0 && objRect.height ==0) continue;
		else processObjectPixels(pt1, pt2);

		cvRectangle(lImg, pt1, pt2, cvScalar(255), 1, 8, 0);
	}

	cvNamedWindow("8-bit", 0);
	cvNamedWindow("Smoothened", 0);
	cvNamedWindow("Thresholded", 0);
	cvNamedWindow("Edge", 0);
	cvNamedWindow("Labelled", 0);

	cvShowImage("8-bit", gImg);
	cvShowImage("Smoothened", sImg);
	cvShowImage("Thresholded", tImg);
	cvShowImage("Edge", eImg);
	cvShowImage("Labelled", lImg);

	cvWaitKey(0);
	cvReleaseImage(& gImg);
	cvReleaseImage(& sImg);
	cvReleaseImage(& tImg);
	cvReleaseImage(& eImg);
	cvReleaseImage(& mImg);
	cvReleaseImage(& lImg);

	cvDestroyAllWindows();
	return 0;
}

double BBSonar::getGlobalThreshold(CvArr* gImg) {
	IplImage *fb, *g, *ginv;
	bool done;
	double T, Tnext;
	double min_val, max_val;
	CvSize size;

	size = cvGetSize(gImg);
	fb = cvCreateImage(size, IPL_DEPTH_8U, 1);
	g = cvCreateImage(size, IPL_DEPTH_8U, 1);
	ginv = cvCreateImage(size, IPL_DEPTH_8U, 1);

	cvMinMaxLoc(gImg, &min_val, &max_val, NULL, NULL, NULL);
	T = 0.5*(min_val + max_val);
	done = false;

	while(!done) {
		cvThreshold(gImg, g, T, 255, CV_THRESH_BINARY);
		cvNot(g, ginv);
		Tnext = 0.5 * (cvAvg(gImg, g).val[0] + cvAvg(gImg, ginv).val[0]);
		done=fabs(T-Tnext)<5;
		T=Tnext;
	}
	return T;
}

int BBSonar::writeIntensities(const IplImage* grayImg) {
	int height, width, intensity;

	intensityOutStream.open(INTENSITY_DATA_PATH.c_str());
	uchar* imgData = (uchar *)grayImg->imageData;

	for (height=0; height<imgHeight; ++height) {
		for (width=0; width < imgWidth; ++width) {
			if (width >= imgWidth) intensity = 0;
			else {
				intensity = ((uchar*)(imgData + imgWidthStep * height))[width];
				if (intensity < GRAYSCALE_THRESH) intensity = 0;
			}
			intensityOutStream << intensity << " ";
		}
		intensityOutStream << "\n";
	}
	intensityOutStream.close();
	return 0;
}

int BBSonar::readIntensities() {
	int height, width;
	uShort imgWord, imgWordTemp;
	char imgWordHi, imgWordLo, imgData8b[imgHeight][imgWidth];
	char newline;

//	read the grayscale intensity values to a 16-bit image
	grayImg16b = cvCreateImage(cvSize(imgWidth, imgHeight), IPL_DEPTH_16U, 1);

	intensityInStream.open(INTENSITY_DATA_PATH.c_str());

	for (height=0; height < imgHeight; ++height) {
		for (width=0; width < imgWidthStep; width+=2){
			intensityInStream >> imgWord;
			imgWordTemp = imgWord;   // Backup temp
			imgWordHi = (imgWord & 0xFF00) >> 8 ;
			imgWordLo = (imgWordTemp & 0x00FF) ;
			grayImg16b->imageData[height * (imgWidthStep) + width] = imgWordHi ;
			grayImg16b->imageData[height * (imgWidthStep) + width +1] = imgWordLo ;

			imgData8b[height][width/2] = imgWordTemp / 65536.0 * 255 ;
		}
		intensityInStream >> newline ;
	}

//	saving the 8-bit image data for processing
	grayImg8b = cvCreateImage(cvSize(imgWidth, imgHeight), IPL_DEPTH_8U, 1);
	for(height=0; height < grayImg8b->height; ++height) {
		for(width=0; width < grayImg8b->width; ++width)
			grayImg8b->imageData[height * grayImg->width + width] = imgData8b[height][width];
	}

	intensityInStream.close();
	return 0;
}


int BBSonar::processObjectPixels(CvPoint pt1, CvPoint pt2) {
	return 0;
}

int BBSonar::writeSonarLog() {
	fson = BVTSonar_Create();
	BVTHead fhead = NULL;

	if (NULL == fson) {
		cout << "error creating file sonar" << endl;
		return -1;
	}
	if((retVal = BVTSonar_CreateFile(fson, SONAR_LOG_PATH.c_str(), son, "")) != 0) {
		cout << "error cloning sonar data to a log: " << BVTError_GetString(retVal) << endl;
		return retVal;
	}
	if((retVal = BVTSonar_GetHead(fson, 0, &fhead)) != 0) {
		cout << "error retrieving head from logged son file: " << BVTError_GetString(retVal) << endl;
		return retVal;
	}
	if((retVal = BVTHead_PutPing(fhead, ping)) != 0) {
		cout << "error putting ping into log file: " << BVTError_GetString(retVal) << endl;
		return retVal;
	}
	BVTSonar_Destroy(fson);
	return 0;
}

int BBSonar::disableTransmission() {
	return BVTHead_SetTxEnable(head, 0);
}


