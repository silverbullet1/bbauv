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

	initSonar();
	setHeadParams();
	// writeIntensities();
	// processImage();
}

Utility::~Utility() {
	if (imgBuffer != NULL) imgBuffer = NULL;
	if (son != NULL) BVTSonar_Destroy(son);
	if (ping != NULL) BVTPing_Destroy(ping);
}

/**
 * initialize the sonar head's connection (ethernet or file interface)
 * shall be used whenever a ping has to be retrieved
 */
int Utility::initSonar() {
	son = BVTSonar_Create();
	retVal = BVTSonar_Open(son, "NET", "192.168.1.45");
//	retVal = BVTSonar_Open(son, "FILE", "data/salmon_small.son");

	if((retVal = BVTSonar_GetHead(son, HEAD_NUM, &head)) != 0) {
		cout << "error retrieving head, exiting now: " << BVTError_GetString(retVal) << endl;
		return retVal;
	}
//	if((retVal = BVTHead_GetPing(head, PING_NUM, &ping)) != 0) {
//		cout << "error retrieving ping: "  << BVTError_GetString(retVal) << endl;
//		return retVal;
//	}
	return 0;
}

/**
 * Get the sonar head's parameter values related to the
 * environment. Set the range and image related parameters
 * along with the obtained environment parameters
 */
int Utility::setHeadParams() {
	startRange = BVTHead_GetStartRange(head);
//	stopRange = BVTHead_GetStopRange(head);
	fluidType = BVTHead_GetFluidType(head);
	soundSpeed = BVTHead_GetSoundSpeed(head);
	analogGain = BVTHead_GetGainAdjustment(head);
	tvGain = BVTHead_GetTVGSlope(head);
	pingCount = BVTHead_GetPingCount(head);
	stopRange = MAX_RANGE;
//	startRange = MIN_RANGE;


	//sets
	if((retVal = BVTHead_SetRange(head, startRange, stopRange)) != 0)
			cout << "error setting range" << endl;

	cout << "(Start, Stop) range:\t" << BVTHead_GetStartRange(head) << "\t" << BVTHead_GetStopRange(head) << endl;

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

/**
 * create a grayscale image for processing from the
 * intensities of the retrieved ping
 */
int Utility::writeIntensities() {
	std::string imgName = "-intensities.png";
	std::string intensitiesName = "-intensities.txt";
    
    if((retVal = BVTHead_GetPing(head, PING_NUM, &ping)) != 0) {
		cout << "error retrieving ping: "  << BVTError_GetString(retVal) << endl;
		return retVal;
	}

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

	// std::string timeString = currentDateTime();
	// std::string grayFile = timeString + imgName;
	// std::string intensitiesFile = timeString + intensitiesName;

	// imwrite(grayFile, matImg);
	imwrite("newIntensities.png", matImg);

	cout << "matImg size: " << "height: " << matImg.rows << "  width: " << matImg.cols << endl;

//	backup data storage in xml format
	// cv::FileStorage storage("store.yml", cv::FileStorage::WRITE);
	// storage << "mat" << matImg;
	// storage.release();

    cv::Mat cImg = imread(COLOR_IMAGE_FILE.c_str(), 0);
	imshow("colorImg", cImg);
	cv::waitKey(3);

	if (NULL != magImg) BVTMagImage_Destroy(magImg);
	if (NULL != colorMap) BVTColorMapper_Destroy(colorMap);
	if (NULL != colorImg) BVTColorImage_Destroy(colorImg);

	return 0;
}

/**
 * apply all the image processing approaches here
 */
int Utility::processImage() {
	grayImg = imread("newIntensities.png", 0);
	cout << "grayImg size: " << grayImg.size() << " [w x h] " << endl;

	Rect roiRect = Rect(Point(0, 0), Point(grayImg.cols, grayImg.rows));
	Mat roiImg = grayImg(roiRect).clone();
	cout << "ROI size: " << roiImg.size()  << " [w x h] " << endl;

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

/* all kinds of thresholding on
 * the sonar image done/called here
 */

//	global thresholding
	double threshVal = getGlobalThreshold(roiImg);
	cout << "global threshold value: " << threshVal << endl;
	cv::threshold(smoothImg, threshImg, threshVal+THRESH_CONSTANT, 255, CV_THRESH_BINARY);

//	OpenCV's adaptive thresholding
//	cv::adaptiveThreshold(smoothImg, threshImg, 255, CV_ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 23, 25);

//	morphology : opening, closing and dilating
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1,1));
	morphologyEx(threshImg, morphOImg, CV_MOP_OPEN, element);
	morphologyEx(threshImg, morphCImg, CV_MOP_CLOSE, element);
	dilate(threshImg, dilatedImg, element);

//	applying canny filter for detecting edges
	Canny(dilatedImg, edgedImg, 1.0, 3.0, 3);

//	cout << "image depth: " << (grayImg.dataend-grayImg.datastart) / (grayImg.cols*grayImg.rows*grayImg.channels()) * 8 << endl;

//	Modified adaptive threshold: derived from OpenCV's adaptive thresholding
	myAdaptiveThreshold(smoothImg, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, 5, 25);

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

/**
 * visualize the intensities of the image using histograms
 * Equalized grayscale image will give a good view
 */
int Utility::drawHistogram() {
	Mat src, equalizedSrc;
	src = imread(INTENSITIES_FILE.c_str(), 0);

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

/**
 * Thresholding approaches applied here
 * Gamma correction to be included first for pool environments
 * Adaptive threshold based on the mean of local pixel intensities implemented
 */
void Utility::myAdaptiveThreshold(Mat gImg, double maxValue, int method,
		int type, int blockSize, double delta) {

	uchar imaxval = saturate_cast<uchar>(maxValue);
	int idelta = type == THRESH_BINARY ? cvCeil(delta) : cvFloor(delta);
	uchar tab[768];

//	grayImg for Gamma correction
	Mat grayImg = gImg.clone();
	Mat powerImg = Mat::zeros(grayImg.size(), CV_8UC1);
	uchar* srcData = grayImg.data;
	uchar* dstData = powerImg.data;

//	applying gamma correction to increase the contrast in the image
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

	if(type == CV_THRESH_BINARY)
		for(int i = 0; i < 768; i++ )
			tab[i] = (uchar)(i - 255 > -idelta ? imaxval : 0);
	else if(type == CV_THRESH_BINARY_INV)
		for(int i = 0; i < 768; i++)
			tab[i] = (uchar)(i - 255 <= -idelta ? imaxval : 0);
	else
		cout << "Unknown/unsupported threshold type applied on the binary image" << endl;

	if(src.isContinuous() && mean.isContinuous() && dst.isContinuous()) {
		size.width *= size.height;
		size.height = 1;
	}

	for(int i = 0; i < size.height; i++ ) {
		const uchar* sdata = src.data + src.step*i;
		const uchar* mdata = mean.data + mean.step*i;
		uchar* ddata = dst.data + dst.step*i;

		for(int j = 0; j < size.width; j++ )
			ddata[j] = tab[sdata[j] - mdata[j] + 255];
	}

	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1,1));
	Mat dstDilated = Mat::zeros(dst.size(), CV_8UC1);
	dilate(dst, dstDilated, element);

	Mat dstEdged = Mat::zeros(dst.size(), CV_8UC1);
	Canny(dstDilated, dstEdged, 1.0, 3.0, 3);

	Mat dstGray = dstEdged.clone();
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
    
	cv::findContours(dstGray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	cout << "number of contours detected: " << contours.size() << endl;
//	if (!contours.size()) {
//		cout << "exiting now since no objects could be detected" << endl;
//		exit(EXIT_FAILURE);
//	}

	Mat labelledImg = Mat::zeros(dstGray.size(), CV_8UC1);
	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect(contours.size());
	RNG rng;

//	polygon and rectangle shapes around the contours are saved in a vector
	for(uInt i = 0; i < contours.size(); i++) {
		approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
		boundRect[i] = boundingRect(Mat(contours_poly[i]));
	}

//	drawing the saved rectangular boxes around the contours
//	and saving their coordinates
	int pointnum = 0;
	// vector<vector<Point> > savedContours;
	// vector<Point> savedPoints;

	for(uInt i = 0; i < contours.size(); i++) {
		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
//		drawContours(labelledImg, contours, i, color, 2, 8, hierarchy, 0, Point());
		drawContours(labelledImg, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());
		if (boundRect[i].area() > CONTOUR_AREA_LOWER_BOUND && boundRect[i].area() < CONTOUR_AREA_UPPER_BOUND) {
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
// 	}
//	Canny(orImg, dstEdged, 1.0, 3.0, 3);
//	imshow("OREdged", dstEdged);

//	imshow("dst", dst);
	// imshow("dstDilated", dstDilated);
	// imshow("dstEdged", dstEdged);
    imshow("labelledImg", labelledImg);

	// imwrite("powerImg.png", powerImg);
	// imwrite("edgedImg.png", dstEdged);
	// imwrite("labelledImg.png", labelledImg);

    waitKey(3);

//	calculate range/bearing of the saved points
	// getRangeBearing();

	return;
}

bool Utility::getRangeBearing() 
{	
    writeIntensities();
    processImage();

  if (magImg == NULL ){
	  cout << "magImg null, can't find range" << endl;
      return false;
  }
  else {
	  for (uInt pointIdx = 0; pointIdx < savedPoints.size(); ++pointIdx) {
		  int x = savedPoints[pointIdx].x;
		  int y = savedPoints[pointIdx].y;
		  cout << "(x  y): " << x << " " << y << endl;	
			// cout << "range: " << BVTMagImage_GetPixelRange(magImg, x, y) << endl;
			// cout << "bearing: " << BVTMagImage_GetPixelRelativeBearing(magImg, x, y) << endl;
			singlePoint.range = BVTMagImage_GetPixelRange(magImg, x, y) ;
			singlePoint.bearing = BVTMagImage_GetPixelRelativeBearing(magImg, x, y) ;
			sonarMsg.rangeBearingVector.push_back(singlePoint);
		}
	}

	savedContours.clear();
	savedPoints.clear();

	return true;
}

/**
 * get a global threshold value
 * Reference: Gonzalez's book on Image Processing
 * works fine only when there are nice peaks in the intensities histogram
 */
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

/**
 * get current date/time in this format: YYYY-MM-DD.HH:mm:ss
 */
inline const std::string Utility::currentDateTime() {
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%m-%d.%X", &tstruct);

    return buf;
}

void Utility::delay(time_t timeout) {
	struct timeval tvDeadline, tvCur;
	gettimeofday(&tvDeadline, NULL);
	gettimeofday(&tvCur, NULL);;
	tvDeadline.tv_sec += timeout;
	
	while(tvCur.tv_sec < tvDeadline.tv_sec) {
		gettimeofday(&tvCur, NULL);
	}
}

int main(int argc, char** argv)
{
	Utility *util = new Utility();
    ROS_INFO("BBSonar invoked");

    ros::init(argc, argv, "BBSonar");
    ros::NodeHandle n;

    ros::Publisher sonarPub = n.advertise<bbauv_msgs::sonarDataVector>("sonarTopic", 1000);
    ros::Rate loopRate(10);
  
    while (ros::ok()) {
        util->getRangeBearing();
        sonarPub.publish(util->sonarMsg);
        ros::spinOnce();
        
        loopRate.sleep();
    }
	return 0;
}
