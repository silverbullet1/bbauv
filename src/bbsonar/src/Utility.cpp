/*
 * Utility.cpp
 *
 *  Created on: Jan 19, 2014
 *      Author: freax
 */

#include "Utility.h"
using namespace cv;


Utility::Utility() : son(NULL), fson(NULL), head(NULL), ping(NULL), rangeData(NULL), magImg(NULL), colorImg(NULL), colorMap(NULL), imgBuffer(NULL), SONAR_PING_RATE(10), enable(true), interfaceOpen(false) {

	retVal = startRange = stopRange = fluidType = soundSpeed = analogGain = tvGain =
	pingCount = imgWidth = imgHeight = imgWidthStep = rangeValCount = 0;

	if (!initSonar()) {
        interfaceOpen = true;
        if (!setHeadParams()) {
            ROS_INFO("BBSonar will be taking pings now");
        }
    }
}

Utility::~Utility() {
	if (imgBuffer != NULL) imgBuffer = NULL;
	if (son != NULL) BVTSonar_Destroy(son);
	if (ping != NULL) BVTPing_Destroy(ping);
    if (NULL != magImg) BVTMagImage_Destroy(magImg);
}

/**
 * initialize the sonar head's connection (ethernet or file interface)
 * shall be used whenever a ping has to be retrieved
 */
int Utility::initSonar() {
    if((son = BVTSonar_Create()) == NULL) {
        ROS_ERROR("could not initialize the sonar head");
    }
    pingSonar("192.168.1.45");
    if((retVal = BVTSonar_Open(son, "NET", "192.168.1.45")) != 0) {
        ROS_ERROR("error opening the sonar interface: %s", BVTError_GetString(retVal));
        return retVal;
    }
	if((retVal = BVTSonar_GetHead(son, HEAD_NUM, &head)) != 0) {
		ROS_ERROR("error retrieving head, exiting now: %s", BVTError_GetString(retVal));
		return retVal;
	}

	return 0;
}

int Utility::pingSonar(char* ip) {
    char buffer[50];
    char c[20];
    sprintf(buffer, "ping -c 3 %s | grep -c ms", ip);
    FILE *p = popen(buffer, "r");
    fgets(c, 5, p);
    pclose(p);
    if (strcmp(c, "5\n") == 0) {
        ROS_INFO("sonar's ip %s is alive", ip);
    }
    else {
        ROS_INFO("sonar's ip %s is not alive", ip);
    }
    
    return 0;
}


/**
 *  get/set sonar parameters
 */
int Utility::setHeadParams() {
	startRange = BVTHead_GetStartRange(head);
//	stopRange = BVTHead_GetStopRange(head);
  	fluidType = BVTHEAD_FLUID_FRESHWATER;   // freshwater fluid type
//  fluidType = BVTHEAD_FLUID_SALTWATER;
	soundSpeed = 1500;  // sound speed in freshwater environment
	analogGain = BVTHead_GetGainAdjustment(head);
	tvGain = BVTHead_GetTVGSlope(head);
	pingCount = BVTHead_GetPingCount(head);
	stopRange = MAX_RANGE;
//	startRange = MIN_RANGE;

	//sets
	if((retVal = BVTHead_SetRange(head, startRange, stopRange)) != 0)
			ROS_ERROR("error setting range");

	ROS_INFO("Start, Stop range: [%f, %f]m", BVTHead_GetStartRange(head), BVTHead_GetStopRange(head));

	if((retVal = BVTHead_SetImageRes(head, RES_TYPE)) != 0)
		ROS_ERROR("error setting image resolution");

	if((retVal = BVTHead_SetImageType(head, IMAGE_TYPE)) != 0)
		ROS_ERROR("error setting image type");

	if((retVal = BVTHead_SetFluidType(head, fluidType)) != 0)
		ROS_ERROR("error setting fluid type");

	if((retVal = BVTHead_SetSoundSpeed(head, soundSpeed)) != 0)
		ROS_ERROR("error setting sound speed");

	if((retVal = BVTHead_SetGainAdjustment(head, analogGain)) != 0)
		ROS_ERROR("error setting analog gain");

	if((retVal = BVTHead_SetTVGSlope(head, tvGain)) != 0)
		ROS_ERROR("error setting TV Gain");
    
	return retVal;
}

/**
 * create a grayscale image for processing from the
 * intensities of the retrieved ping
 */
int Utility::writeIntensities() {
    if((retVal = BVTHead_GetPing(head, PING_NUM, &ping)) != 0) {
		ROS_ERROR("error retrieving ping: %s", BVTError_GetString(retVal));
		return retVal;
	}

	if((retVal = BVTPing_GetImage(ping, &magImg)) != 0) {
		ROS_ERROR("error converting ping: %s", BVTError_GetString(retVal));
		return retVal;
	}

	BVTMagImage_SavePGM(magImg, GRAYSCALE_MAG_FILE.c_str());
	if((colorMap = BVTColorMapper_Create()) == NULL) {
		ROS_ERROR("error creating color mapper");
		return -1;
	}
	if((retVal = BVTColorMapper_Load(colorMap, COLOR_MAPPER_PATH.c_str())) != 0) {
		ROS_ERROR("error retrieving color map: %s", BVTError_GetString(retVal));
		return retVal;
	}
	if((retVal = BVTColorMapper_MapImage(colorMap, magImg, &colorImg)) != 0) {
		ROS_ERROR("error mapping to color image: %s", BVTError_GetString(retVal));
		return retVal;
	}
	BVTColorImage_SavePPM(colorImg, COLOR_IMAGE_FILE.c_str());
    
//	ROS_INFO("magImg resolution:  [%d, %d]", BVTMagImage_GetWidth(magImg), BVTMagImage_GetHeight(magImg));
    
//    ROS_INFO("Range resolution: %lf", BVTColorImage_GetRangeResolution(colorImg));

	imgBuffer = BVTMagImage_GetBits(magImg);
	if (imgBuffer == NULL) cout << "imgBuf null" << endl;
	matImg = Mat(BVTMagImage_GetHeight(magImg), BVTMagImage_GetWidth(magImg), CV_16UC1, imgBuffer);

	imwrite(INTENSITIES_FILE.c_str(), matImg);

    cv::Mat cImg = imread(COLOR_IMAGE_FILE.c_str(), 0);
    outImg = cImg.clone();
    resize(outImg, outImg, Size(640, 480));

	if (NULL != colorMap) BVTColorMapper_Destroy(colorMap);
	if (NULL != colorImg) BVTColorImage_Destroy(colorImg);

	return 0;
}

/**
 * apply all the image processing approaches here
 */
int Utility::processImage() {
	grayImg = imread(INTENSITIES_FILE.c_str(), 0);
//	cout << "grayImg size: " << grayImg.size() << " [w x h] " << endl;

	Rect roiRect = Rect(Point(0, 0), Point(grayImg.cols, grayImg.rows));
	Mat roiImg = grayImg(roiRect).clone();
//	cout << "ROI size: " << roiImg.size()  << " [w x h] " << endl;

	cv::Mat smoothImg, threshImg, edgedImg, morphOImg, morphCImg, dilatedImg;

//	initializing the image matrices with zeros
	smoothImg = Mat::zeros(roiImg.rows, roiImg.cols, CV_8UC1);
	threshImg = Mat::zeros(roiImg.rows, roiImg.cols, CV_8UC1);
	edgedImg = Mat::zeros(roiImg.rows, roiImg.cols, CV_8UC1);
	morphOImg = Mat::zeros(roiImg.rows, roiImg.cols, CV_8UC1);
	morphCImg = Mat::zeros(roiImg.rows, roiImg.cols, CV_8UC1);
	dilatedImg = Mat::zeros(roiImg.rows, roiImg.cols, CV_8UC1);

//	smoothened
	cv::medianBlur(roiImg, smoothImg, 3);

//	global thresholding : not being used for our operations
//	double threshVal = getGlobalThreshold(roiImg);
//	cout << "global threshold value: " << threshVal << endl;
//	cv::threshold(smoothImg, threshImg, threshVal+THRESH_CONSTANT, 255, CV_THRESH_BINARY);

//	OpenCV's adaptive thresholding
//	cv::adaptiveThreshold(smoothImg, threshImg, 255, CV_ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 23, 25);

//	morphology : opening, closing and dilating
//	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1,1));
//	morphologyEx(threshImg, morphOImg, CV_MOP_OPEN, element);
//	morphologyEx(threshImg, morphCImg, CV_MOP_CLOSE, element);
//	dilate(threshImg, dilatedImg, element);

//	applying canny filter for detecting edges
//	Canny(dilatedImg, edgedImg, 1.0, 3.0, 3);

//	cout << "image depth: " << (grayImg.dataend-grayImg.datastart) / (grayImg.cols*grayImg.rows*grayImg.channels()) * 8 << endl;

//	Modified adaptive threshold: derived from OpenCV's adaptive thresholding
	myAdaptiveThreshold(smoothImg, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, 5, 25);

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
		}
	}

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

	labelledImg = Mat::zeros(dstGray.size(), CV_8UC1);
	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect(contours.size());
	RNG rng;

//	polygon and rectangle shapes around the contours are saved in a vector
	for(uInt i = 0; i < contours.size(); i++) {
		approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
		boundRect[i] = boundingRect(Mat(contours_poly[i]));
	}

//	drawing the saved rectangular boxes around the contours and saving their coordinates
	int pointnum = 0;

	for(uInt i = 0; i < contours.size(); i++) {
		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		drawContours(labelledImg, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());
		if (boundRect[i].area() > CONTOUR_AREA_LOWER_BOUND && boundRect[i].area() < CONTOUR_AREA_UPPER_BOUND) {
			savedContours.push_back(contours[i]);
			++pointnum;
			rectangle(labelledImg, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
			cout << pointnum << ": " << boundRect[i].tl() << " " << boundRect[i].br() << " " << boundRect[i].area() << endl;
			savedPoints.push_back(boundRect[i].br());
		}
	}
    
//  resizing, for it to be of equal dimension as the raw image in the UI
//    resize(labelledImg, labelledImg, Size(640, 480));

	return;
}


bool Utility::getRangeBearing() 
{
    writeIntensities();
//    processImage();

    if (magImg == NULL){
	  ROS_ERROR("Magnitude image null, can't find range");
      return false;
    }
    return true;
}


void Utility::processFilterImage(const sensor_msgs::ImageConstPtr& source) {
    Mat gradX, gradY;
    Mat absGradX, absGradY;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;

    cv_bridge::CvImagePtr cvImg;
    cvImg = cv_bridge::toCvCopy(source);
    filterImg = cvImg->image.clone();
    
    GaussianBlur(filterImg, filterImg, Size(3,3), 0, 0, BORDER_DEFAULT);
    cvtColor(filterImg, filterImg, CV_BGR2GRAY);
    
//  resizing it to magImage resolution, for getting range-bearing of the targets
    resize(filterImg, filterImg, Size(BVTMagImage_GetWidth(magImg), BVTMagImage_GetHeight(magImg)));

    vector<Vec4i> lines;
    HoughLinesP(filterImg, lines, 1, CV_PI/180, 50, 50, 10);
    if (!lines.size()) {
        ROS_INFO("No objects detected");
    }
    for(size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];
//        line(filterImg, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
        Point firstPoint, secondPoint, midPoint;
        firstPoint = Point(l[0], l[1]);
        secondPoint = Point(l[2], l[3]);
        midPoint = (firstPoint + secondPoint) * 0.5 ;
        savedPoints.push_back(midPoint);
    }
    
    return;
}


bool Utility::getPixelRangeBearing(bbauv_msgs::sonar_pixel::Request &req, bbauv_msgs::sonar_pixel::Response &rsp)
{
    int row = ((double)(req.y) / 480) * BVTMagImage_GetHeight(magImg);
    int col = ((double)(req.x) / 640) * BVTMagImage_GetWidth(magImg);
    
    rsp.range = BVTMagImage_GetPixelRange(magImg, (int)row, (int)col);
    rsp.bearing = BVTMagImage_GetPixelRelativeBearing(magImg, (int)row, (int)col);
        
//    ROS_INFO("range bearing for pixel [%d  %d] = [%f  %f]", req.x, req.y, rsp.range, rsp.bearing);
    
    return true;
}

/**
 * get a global threshold value
 * works fine only when there are nice peaks in the intensities histogram
 */
double Utility::getGlobalThreshold(Mat gImg) {
	cv::Mat g, ginv;
	bool done;
	double T, Tnext;
	double min_val, max_val;

	cv::minMaxLoc(gImg, &min_val, &max_val, 0, 0, noArray());

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

bool Utility::enableSonar(bbauv_msgs::sonar_switch::Request &req, bbauv_msgs::sonar_switch::Response &rsp) {
    ROS_INFO("sonar_switch pressed");
    if (req.enable) {
        enable = true;
        rsp.isEnabled = true;
        ROS_INFO("BBSonar enabled");
    }
    else {
        enable = false;
        rsp.isEnabled = false;
        ROS_INFO("BBSonar disabled");
    }
    
    return true;
}

int main(int argc, char** argv)
{
    ROS_INFO("BBSonar invoked");
    Utility *util = new Utility();
    
    ros::init(argc, argv, "bbsonar");
    ros::NodeHandle nHandle;
    cv_bridge::CvImage cvImg;
    
    ros::Publisher imagePub = nHandle.advertise<sensor_msgs::Image>("sonar_image", 1);
    
//    ros::Publisher sonarDataPub = nHandle.advertise<bbauv_msgs::sonar_data_vector>("sonar_data_vector", 1000);
    
//    ros::Publisher sonarDataPub = nHandle.advertise<bbauv_msgs::sonar_data>("sonar_data", 1000);

    ros::ServiceServer sonarSwitchService = nHandle.advertiseService("sonar_switch", &Utility::enableSonar, util);
    
    ros::ServiceServer sonarPixelService = nHandle.advertiseService("sonar_pixel", &Utility::getPixelRangeBearing, util);
    
    ros::Rate loopRate(util->SONAR_PING_RATE);
    while (ros::ok()) {
        if (util->enable && util->interfaceOpen) {
            cvImg.encoding = sensor_msgs::image_encodings::MONO8;
            cvImg.image = util->outImg;
            imagePub.publish(cvImg.toImageMsg());
            
            util->getRangeBearing();
    
//            sonarDataPub.publish(util->sonarMsg);
//            sonarDataPub.publish(util->singlePoint);
        }
        else {
//            ROS_WARN("BBSonar is switched off, not taking pings");
        }

        ros::spinOnce();
        loopRate.sleep();
    }

	return 0;
}
