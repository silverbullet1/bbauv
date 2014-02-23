/*
 * Utility.cpp
 *
 *  Created on: Jan 19, 2014
 *      Author: freax
 */

#include "Utility.h"
using namespace cv;

Utility::Utility() : son(NULL), fson(NULL), head(NULL), ping(NULL), rangeData(NULL), magImg(NULL), colorImg(NULL),
		colorMap(NULL), imgBuffer(NULL) {

	retVal = startRange = stopRange = fluidType = soundSpeed = analogGain = tvGain =
	pingCount = imgWidth = imgHeight = imgWidthStep = rangeValCount = 0;
}

Utility::~Utility() {
	if (imgBuffer != NULL) imgBuffer = NULL;
	if (son != NULL) BVTSonar_Destroy(son);
	if (ping != NULL) BVTPing_Destroy(ping);
}

int Utility::initSonar() {
	son = BVTSonar_Create();
	retVal = BVTSonar_Open(son, "NET", "192.168.1.45");
//	retVal = BVTSonar_Open(son, "FILE", "data/salmon_small.son");

	if((retVal = BVTSonar_GetHead(son, HEAD_NUM, &head)) != 0) {
		cout << "error retrieving head, exiting now: " << BVTError_GetString(retVal) << endl;
		return retVal;
	}
	if((retVal = BVTHead_GetPing(head, PING_NUM, &ping)) != 0) {
		cout << "error retrieving ping: "  << BVTError_GetString(retVal) << endl;
		return retVal;
	}
	return 0;
}

int Utility::setHeadParams() {
	startRange = BVTHead_GetStartRange(head);
//	stopRange = BVTHead_GetStopRange(head);
	fluidType = BVTHead_GetFluidType(head);
	soundSpeed = BVTHead_GetSoundSpeed(head);
	analogGain = BVTHead_GetGainAdjustment(head);
	tvGain = BVTHead_GetTVGSlope(head);
	pingCount = BVTHead_GetPingCount(head);
//	stopRange = MAX_RANGE;
	startRange = 2;

	cout << "(Start, Stop) range:\t" << BVTHead_GetStartRange(head) << "\t" << BVTHead_GetStopRange(head) << endl;

	//sets
	if((retVal = BVTHead_SetRange(head, startRange, stopRange)) != 0)
			cout << "error setting range" << endl;

	if((retVal = BVTHead_SetImageRes(head, RES_TYPE)) != 0)
		cout << "error setting image resolution" << endl;

	if((retVal = BVTHead_SetImageType(head, IMAGE_TYPE)) != 0)
		cout << "error setting image type" << endl;

	if((retVal = BVTHead_SetFluidType(head, fluidType)) != 0)
		cout << "error setting fluid type" << endl;

	if((retVal = BVTHead_SetSoundSpeed(head, soundSpeed)) != 0)
		cout << "error setting sound speed" << endl;

	if((retVal = BVTHead_SetGainAdjustment(head, analogGain)) != 0)
		cout << "error setting analog gain" << endl;

	if((retVal = BVTHead_SetTVGSlope(head, tvGain)) != 0)
		cout << "error setting TV Gain" << endl;

	return 0;
}

int Utility::writeIntensities() {
	std::string imgName = "-intensities.png";
	std::string intensitiesName = "-intensities.txt";

	if((retVal = BVTPing_GetImage(ping, &magImg)) != 0) {
		cout << "error retrieving ping: " << BVTError_GetString(retVal) << endl;
		return retVal;
	}

	BVTMagImage_SavePGM(magImg, GRAYSCALE_MAG_FILE.c_str());
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
	BVTColorImage_SavePPM(colorImg, COLOR_IMAGE_FILE.c_str());
	cout << "magImg size: " << "height: " << BVTMagImage_GetHeight(magImg) << "  width: " << BVTMagImage_GetWidth(magImg) <<  endl;

	imgBuffer = BVTMagImage_GetBits(magImg);
	if (imgBuffer == NULL) cout << "imgBuf null" << endl;
	matImg = Mat(BVTMagImage_GetHeight(magImg), BVTMagImage_GetWidth(magImg), CV_16UC1, imgBuffer);
//	cout  << "matImg depth, channel: " << matImg.depth() << " " << matImg.channels() << endl;

	std::string timeString = currentDateTime();
	std::string grayFile = timeString + imgName;
	std::string intensitiesFile = timeString + intensitiesName;

	imwrite("newIntensities.png", matImg);
	imwrite(grayFile, matImg);

	cout << "matImg size: " << "height: " << matImg.rows << "  width: " << matImg.cols << endl;

//	ofstream iOut(intensitiesFile.c_str());
//	for(int row=0; row < matImg.rows; ++row) {
//		for(int col=0; col < matImg.cols; ++col) {
//			cv::Scalar intensity = matImg.at<uchar>(col, row);
//			uInt intensityVal = (uInt)intensity.val[0];
//			intensityVal = intensityVal > (uInt)GRAYSCALE_THRESH ? intensityVal : 0;
//			iOut << intensityVal;
//			iOut << " " ;
//		}
//		iOut << endl;
//	}
//	iOut.close();

//	backup data storage in xml format
	cv::FileStorage storage("store.yml", cv::FileStorage::WRITE);
	storage << "mat" << matImg;
	storage.release();

//	imshow("matImg", matImg);
//	cv::waitKey(0);

	if (NULL != magImg) BVTMagImage_Destroy(magImg);
	if (NULL != colorMap) BVTColorMapper_Destroy(colorMap);
	if (NULL != colorImg) BVTColorImage_Destroy(colorImg);

	return 0;
}

int Utility::processImage() {
//	ifstream intensityIn;

//	hardcoded read : reading the image from the stored xml file
//	cv::Mat grayImg = Mat::zeros(BVTMagImage_GetHeight(magImg), BVTMagImage_GetWidth(magImg), CV_16UC1);
//	cv::FileStorage storage("store.yml", cv::FileStorage::READ);
//	storage["mat"] >> grayImg;
//	storage.release();

//	hardcoded read : beware of the image's path
//	cv::Mat grayImg = Mat::zeros(316, 250, CV_16UC1);
	grayImg = imread("newIntensities.png", 0);
	cout << "grayImg size: " << grayImg.size() << " [w x h] " << endl;

	Rect roiRect = Rect(Point(0, 55), Point(grayImg.cols, grayImg.rows - ROWS_CROPPED));
	Mat roiImg = grayImg(roiRect).clone();
	cout << "ROI size: " << roiImg.size()  << " [w x h] " << endl;

//	all processing stuffs with the saved grayscale image
//	cv::Mat grayImg = matImg.clone();
	cv::Mat smoothImg, threshImg, edgedImg, morphOImg, morphCImg, dilatedImg, labelledImg;

//	initializing the image matrices with zeros
	smoothImg = Mat::zeros(roiImg.rows, roiImg.cols, CV_8UC1);
	threshImg = Mat::zeros(roiImg.rows, roiImg.cols, CV_8UC1);
	edgedImg = Mat::zeros(roiImg.rows, roiImg.cols, CV_8UC1);
	morphOImg = Mat::zeros(roiImg.rows, roiImg.cols, CV_8UC1);
	morphCImg = Mat::zeros(roiImg.rows, roiImg.cols, CV_8UC1);
	dilatedImg = Mat::zeros(roiImg.rows, roiImg.cols, CV_8UC1);

//	smoothened
	cv::medianBlur(roiImg, smoothImg, 3);

	double threshVal = getGlobalThreshold(roiImg);
//	double threshVal = 200;
	cout << "global threshold value: " << threshVal << endl;
	cv::threshold(smoothImg, threshImg, threshVal+THRESH_CONSTANT, 255, CV_THRESH_BINARY);
//	cv::adaptiveThreshold(smoothImg, threshImg, 255, CV_ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 23, 25);

//	morphology : opening, closing and dilating
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1,1));
	morphologyEx(threshImg, morphOImg, CV_MOP_OPEN, element);
	morphologyEx(threshImg, morphCImg, CV_MOP_CLOSE, element);
	dilate(threshImg, dilatedImg, element);

//	applying canny filter for detecting edges
	Canny(dilatedImg, edgedImg, 1.0, 3.0, 3);

//	cout << "image depth: " << (grayImg.dataend-grayImg.datastart) / (grayImg.cols*grayImg.rows*grayImg.channels()) * 8 << endl;

	myAdaptiveThreshold(smoothImg, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, 5, 25);

////	labelled image
//	RNG rng;
//	vector<vector<Point> > contours;
//	vector<Vec4i> hierarchy;
//	findContours(adaptiveImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
//	labelledImg = Mat::zeros(adaptiveImg.size(), CV_16UC1);
//	for(int i = 0; i< contours.size(); i++) {
//		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
//		drawContours(labelledImg, contours, i, color, 2, 8, hierarchy, 0, Point());
//	}

//	imshow("grayImg", grayImg);
//	imshow("ROI", roiImg);
//	imshow("smoothened", smoothImg);
//	imshow("thresholded", threshImg);
//	imshow("morphOpened", morphOImg);
//	imshow("morphClosed", morphCImg);
//	imshow("dilated", dilatedImg);
//	imshow("edged", edgedImg);
//	imshow("labelled", labelledImg);

//	imwrite("edged.png", edgedImg);
//	waitKey(0);

	return 0;
}

int Utility::drawHistogram() {
	Mat src, equalizedSrc;
	src = imread("newIntensities.png", 0);

//	without using opencv's calcHist method
//    int bins = 256;             // number of bins
//    int nc = src.channels();  	// number of channels
//    vector<Mat> hist(nc);       // array for storing the histograms
//    vector<Mat> canvas(nc);    	// images for displaying the histogram
//    int hmax[3] = {0,0,0};     	// peak value for each histogram
//
//    for (int i = 0; i < hist.size(); i++)
//        hist[i] = Mat::zeros(1, bins, CV_32SC1);
//
//    for (int i = 0; i < src.rows; i++) {
//        for (int j = 0; j < src.cols; j++) {
//            for (int k = 0; k < nc; k++) {
//                uchar val = nc == 1 ? src.at<uchar>(i,j) : src.at<Vec3b>(i,j)[k];
//                hist[k].at<int>(val) += 1;
//            }
//        }
//    }
//
//    for (int i = 0; i < nc; i++) {
//        for (int j = 0; j < bins-1; j++)
//            hmax[i] = hist[i].at<int>(j) > hmax[i] ? hist[i].at<int>(j) : hmax[i];
//    }
//
//    const char* wname[3] = { "blue", "green", "red" };
//    Scalar colors[3] = { Scalar(255,0,0), Scalar(0,255,0), Scalar(0,0,255) };
//
//    for (int i = 0; i < nc; i++) {
//        canvas[i] = Mat::ones(125, bins, CV_8UC3);
//        for (int j = 0, rows = canvas[i].rows; j < bins-1; j++) {
//            line(
//                canvas[i],
//                Point(j, rows),
//                Point(j, rows - (hist[i].at<int>(j) * rows/hmax[i])),
//                nc == 1 ? Scalar(200,200,200) : colors[i],
//                1, 8, 0
//            );
//        }
//        namedWindow("histogram", 0);
////      imshow(nc == 1 ? "value" : wname[i], canvas[i]);
//        imshow("histogram", canvas[i]);
//    }

	if(!src.data)
		return -1;

//	equalized image
	equalizeHist(src, equalizedSrc);

	vector<Mat> planes, eqPlanes;
	split(src, planes);
	split(equalizedSrc, eqPlanes);

	int histSize = 256;
	float range[] = {0, 256} ;
	const float* histRange = { range };

	bool uniform = true; bool accumulate = false;
	Mat grayHist, grayEqHist;

	calcHist(&planes[0], 1, 0, Mat(), grayHist, 1, &histSize, &histRange, uniform, accumulate);
	calcHist(&eqPlanes[0], 1, 0, Mat(), grayEqHist, 1, &histSize, &histRange, uniform, accumulate);
	int histWidth = 512; int histHeight = 400;
	int bin_w = cvRound((double) histWidth/histSize );

	Mat histImage(histHeight, histWidth, CV_8UC3, Scalar(0,0,0));
	Mat histEqImage(histHeight, histWidth, CV_8UC3, Scalar(0,0,0));

	normalize(grayHist, grayHist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
	normalize(grayEqHist, grayEqHist, 0, histEqImage.rows, NORM_MINMAX, -1, Mat());

	for( int i = 1; i < histSize; i++ )
	{
	    line (	histImage,
	    		Point(bin_w*(i-1), histHeight - cvRound(grayHist.at<float>(i-1))),
	    		Point(bin_w*(i), histHeight - cvRound(grayHist.at<float>(i))),
	    		Scalar(255, 255, 0), 2, 8, 0
	    	  );

	    line (	histEqImage,
	   	    	Point(bin_w*(i-1), histHeight - cvRound(grayEqHist.at<float>(i-1))),
	   	    	Point(bin_w*(i), histHeight - cvRound(grayEqHist.at<float>(i))),
	   	    	Scalar(255, 255, 0), 2, 8, 0
	    	  );
	}

	namedWindow("histogram", CV_WINDOW_AUTOSIZE);
	namedWindow("equalized histogram", CV_WINDOW_AUTOSIZE);
//	imshow("histogram", histImage);
//	imshow("equalized histogram", histEqImage);

    waitKey(0);
	return 0;
}

double Utility::getGlobalThreshold(Mat gImg) {
	cv::Mat g, ginv;
	bool done;
	double T, Tnext;
	double min_val, max_val;

	cv::minMaxLoc(gImg, &min_val, &max_val, 0, 0, noArray());
	cout << "Min val: " << min_val << "\tMax val: " << max_val << endl;

	T = 0.5 * (min_val + max_val);
	done = false;

	while(!done) {
		cv::threshold(gImg, g, T, 255, CV_THRESH_BINARY);
		cv::bitwise_not(g, ginv);
		Tnext = 0.5 * (cv::mean(gImg, g).val[0] + cv::mean(gImg, ginv).val[0]);
		done = fabs(T-Tnext) < 100;
		T = Tnext;
	}

	return T;
}

void Utility::myAdaptiveThreshold(Mat gImg, double maxValue, int method,
		int type, int blockSize, double delta) {

	cout << "Adaptive thresholding" << endl;
//	grayImg for Gamma correction
	Mat grayImg = gImg.clone();
	Mat powerImg = Mat::zeros(grayImg.size(), CV_8UC1);
	uchar* srcData = grayImg.data;
	uchar* dstData = powerImg.data;

	for (int i=0; i < grayImg.rows; ++i) {
		for (int j=0; j < grayImg.cols; ++j) {
			dstData[i * grayImg.step + j] = PL_CONST * pow(srcData[i * grayImg.step + j], PL_GAMMA);
//			Scalar s = grayImg.at<uchar>(Point(i, j));
//			uInt intensity = (unsigned int)s.val[0];
		}
	}

	cv::FileStorage storage("powerStore.yml", cv::FileStorage::WRITE);
	storage << "mat" << powerImg;
	storage.release();

//	imshow("powerImg", powerImg);

//	src image for adaptive thresholding
	Mat src = powerImg.clone();
	Mat mean;
	Mat dst = Mat::zeros(src.size(), CV_8UC1);
	Size size = src.size();

	if(src.data != dst.data)
		mean = dst;

	if(method == ADAPTIVE_THRESH_MEAN_C)
		boxFilter(src, mean, src.type(), Size(blockSize, blockSize),
				Point(-1,-1), true, BORDER_REPLICATE);
	else if(method == ADAPTIVE_THRESH_GAUSSIAN_C)
		GaussianBlur(src, mean, Size(blockSize, blockSize), 0, 0, BORDER_REPLICATE);

	int i, j;
	uchar imaxval = saturate_cast<uchar>(maxValue);
	int idelta = type == THRESH_BINARY ? cvCeil(delta) : cvFloor(delta);
	uchar tab[768];

	if(type == CV_THRESH_BINARY)
		for( i = 0; i < 768; i++ )
			tab[i] = (uchar)(i - 255 > -idelta ? imaxval : 0);
	else if(type == CV_THRESH_BINARY_INV)
		for(i = 0; i < 768; i++)
			tab[i] = (uchar)(i - 255 <= -idelta ? imaxval : 0);
	else
		CV_Error(CV_StsBadFlag, "Unknown/unsupported threshold type");

	if(src.isContinuous() && mean.isContinuous() && dst.isContinuous()) {
//		cout << "continous!!" << endl;
		size.width *= size.height;
		size.height = 1;
	}

	for( i = 0; i < size.height; i++ ) {
		const uchar* sdata = src.data + src.step*i;
		const uchar* mdata = mean.data + mean.step*i;
		uchar* ddata = dst.data + dst.step*i;

		for( j = 0; j < size.width; j++ )
			ddata[j] = tab[sdata[j] - mdata[j] + 255];
	}

	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1,1));
	Mat dstDilated = Mat::zeros(dst.size(), CV_8UC1);
	dilate(dst, dstDilated, element);

	Mat dstEdged = Mat::zeros(dst.size(), CV_8UC1);
	Canny(dstDilated, dstEdged, 1.0, 3.0, 3);
	imwrite("edgedImg.png", dstEdged);

	Mat dstGray = dstEdged.clone();
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	imshow("dstGray", dstGray);
	cv::findContours(dstGray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	cout << "number of contours: " << contours.size() << endl;

	Mat labelledImg = Mat::zeros(dstGray.size(), CV_8UC1);
	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect(contours.size());
	RNG rng;

//	polygon and rectangle shapes around the contours
	for(uInt i = 0; i < contours.size(); i++) {
		approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
		boundRect[i] = boundingRect(Mat(contours_poly[i]));
	}

//	saving the object's rectangular coordinates
	int pointnum = 0;
	vector<vector<Point> > savedContours;
	vector<Point> savedPoints;
	for(uInt i = 0; i < contours.size(); i++) {
		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
//		drawContours(labelledImg, contours, i, color, 2, 8, hierarchy, 0, Point());
		drawContours(labelledImg, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());
		if (boundRect[i].area() > CONTOUR_AREA_THRESHOLD) {
			savedContours.push_back(contours[i]);
			++pointnum;
//			rectangle(dstDilated, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
			rectangle(labelledImg, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
			cout << pointnum << ": " << boundRect[i].tl() << " " << boundRect[i].br() << " " << boundRect[i].area() << endl;
			savedPoints.push_back(boundRect[i].br());
		}
	}

//	Boolean operations on src and dst image to be experimented here
//	Mat orImg = Mat::zeros(dst.size(), CV_8UC1);
//	for( i = 0; i < size.height; i++ ) {
//		const uchar* srcData = srcDilated.data + srcDilated.step*i ;
//		const uchar* dstData = dstDilated.data + dstDilated.step*i ;
//		uchar* andData = orImg.data + orImg.step*i ;
//
//		for( j = 0; j < size.width; j++ ) {
//			andData[j] = srcData[j] * dstData[j] ;
//		}
//	}
//	Canny(orImg, dstEdged, 1.0, 3.0, 3);
//	imshow("OREdged", dstEdged);

//	imshow("dst", dst);
	imshow("dstDilated", dstDilated);
	imshow("dstEdged", dstEdged);
	imshow("labelledImg", labelledImg);
	imwrite("labelledImg.png", labelledImg);

	waitKey(0);

//	calculate range/bearing of the saved points
	getRangeBearing(savedPoints);

	return;
}

void Utility::getRangeBearing(vector<cv::Point> savedPoints) {
	for (uInt pointIdx=0; pointIdx < savedPoints.size(); ++pointIdx) {
		int x = savedPoints[pointIdx].x;
		int y = savedPoints[pointIdx].y;
		cout << "(x  y): " << x << " " << y << endl;
		if (magImg == NULL)
			cout << "magImg null, can't find range" << endl;
		else {
			cout << "range: " << BVTMagImage_GetPixelRange(magImg, x, y) << endl;
			cout << "bearing: " << BVTMagImage_GetPixelRelativeBearing(magImg, x, y) << endl;
		}
	}
}

// 	Get current date/time, format is YYYY-MM-DD.HH:mm:ss
inline const std::string Utility::currentDateTime() {
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%m-%d.%X", &tstruct);

    return buf;
}

void Utility::delay(long delay)
{
	clock_t start = clock();
	while(clock() - start < delay);
}
