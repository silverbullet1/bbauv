/*
 * Utility.cpp
 *
 *  Created on: Jan 19, 2014
 *      Author: freax
 */

#include "Utility.h"
using namespace cv;


Utility::Utility() : son(NULL), fson(NULL), head(NULL), magImg(NULL), SONAR_PING_RATE(10), enable(true), interfaceOpen(false) {

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
	if (son != NULL) BVTSonar_Destroy(son);
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
    BVTPing ping;
    if((retVal = BVTHead_GetPing(head, PING_NUM, &ping)) != 0) {
		ROS_ERROR("error retrieving ping: %s", BVTError_GetString(retVal));
		return retVal;
	}
    
//    BVTMagImage magImg;
	if((retVal = BVTPing_GetImage(ping, &magImg)) != 0) {
		ROS_ERROR("error converting ping: %s", BVTError_GetString(retVal));
		return retVal;
	}

	BVTColorMapper colorMap;
//	BVTMagImage_SavePGM(magImg, GRAYSCALE_MAG_FILE.c_str());
	if((colorMap = BVTColorMapper_Create()) == NULL) {
		ROS_ERROR("error creating color mapper");
		return -1;
	}
	if((retVal = BVTColorMapper_Load(colorMap, COLOR_MAPPER_PATH.c_str())) != 0) {
		ROS_ERROR("error retrieving color map: %s", BVTError_GetString(retVal));
		return retVal;
	}
    
    BVTColorImage colorImg;
	if((retVal = BVTColorMapper_MapImage(colorMap, magImg, &colorImg)) != 0) {
		ROS_ERROR("error mapping to color image: %s", BVTError_GetString(retVal));
		return retVal;
	}
    
    int height = BVTColorImage_GetHeight(colorImg);
    int width = BVTColorImage_GetWidth(colorImg);

    Mat cImg(Size(width, height), CV_8UC4, BVTColorImage_GetBits(colorImg));
    cvtColor(cImg, cImg, CV_RGBA2RGB);
    cvtColor(cImg, cImg, CV_RGB2GRAY);
    outImg = cImg.clone();
    resize(outImg, outImg, Size(640, 480));
    
    BVTPing_Destroy(ping);
	BVTColorMapper_Destroy(colorMap);
	BVTColorImage_Destroy(colorImg);
    
	return 0;
}


bool Utility::getPixelRangeBearing(bbauv_msgs::sonar_pixel::Request &req, bbauv_msgs::sonar_pixel::Response &rsp)
{
    int row = ((double)(req.y) / 480) * BVTMagImage_GetHeight(magImg);
    int col = ((double)(req.x) / 640) * BVTMagImage_GetWidth(magImg);
    
    rsp.range = BVTMagImage_GetPixelRange(magImg, (int)row, (int)col);
    rsp.bearing = BVTMagImage_GetPixelRelativeBearing(magImg, (int)row, (int)col);
    
    ROS_INFO("range bearing for pixel [%d  %d] = [%f  %f]", req.x, req.y, rsp.range, rsp.bearing);
    
    BVTMagImage_Destroy(magImg);
    return true;
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

    ros::ServiceServer sonarSwitchService = nHandle.advertiseService("sonar_switch", &Utility::enableSonar, util);
    
    ros::ServiceServer sonarPixelService = nHandle.advertiseService("sonar_pixel", &Utility::getPixelRangeBearing, util);
    
    ros::Rate loopRate(util->SONAR_PING_RATE);
    while (ros::ok()) {
        if (util->enable && util->interfaceOpen) {
            util->writeIntensities();
            cvImg.encoding = sensor_msgs::image_encodings::MONO8;
            cvImg.image = util->outImg;
            imagePub.publish(cvImg.toImageMsg());
            util->outImg.release();
        }
        else {
//            ROS_WARN("BBSonar is switched off, not taking pings");
        }
        ros::spinOnce();
        loopRate.sleep();
    }

	return 0;
}
