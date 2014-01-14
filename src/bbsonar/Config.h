/*
 * Config.h
 *
 *  Created on: Dec 17, 2013
 *      Author: freax
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#ifndef ETH_SONAR
	#define ETH_SONAR
	#define LOG_SONAR_DATA
	#define USE_XY_IMAGE
	#define SAVE_SONAR_IMAGES
	#define READ_INTENSITIES
	const int HEAD_NUM = 1;
	const int PING_NUM = -1;
#else
	#define READ_SONAR_LOG
	const int HEAD_NUM = 0;
	const int PING_NUM = 0;
#endif

#ifdef SAVE_SONAR_IMAGES
	const std::string GRAYSCALE_IMAGE_FILE = "grayImage.pgm" ;
	const std::string COLOR_IMAGE_FILE = "colorImage.ppm" ;
#endif

#define IMAGE_TYPE	BVTHEAD_IMAGE_XY		// option(s): XY, RTHETA
#define RES_TYPE	BVTHEAD_RES_LOW 		// option(s): OFF, LOW, MED, HIGH, AUTO

const std::string SONAR_LOG_PATH = "/home/freax/bbsonar/sonarLog.son" ;
const std::string COLOR_MAPPER_PATH = "/home/freax/bvtsdk/colormaps/bone.cmap" ;
const std::string RANGE_DATA_PATH = "/home/freax/bbsonar/rangeData.txt";
const std::string BEARING_DATA_PATH = "/home/freax/bbsonar/bearingData.txt";
const std::string INTENSITY_DATA_PATH = "/home/freax/bbsonar/intensityData.txt";

const int RES_IN_METRES = 20;
const int MIN_RANGE = 2;
const int MAX_RANGE = 20;
const int GRAYSCALE_THRESH = 150;
const int HOTSPOT_THRESH = 150;
const unsigned short NOISE_THRESHOLD = 2000;

#endif /* CONFIG_H_ */