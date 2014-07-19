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
    const std::string INTENSITIES_FILE = "/home/bbauvsbc1/bbauv/src/bbsonar/newIntensities.png";
	const std::string GRAYSCALE_MAG_FILE = "/home/bbauvsbc1/bbauv/src/bbsonar/grayImage.pgm" ;
	const std::string GRAYSCALE_IMAGE_FILE = "/home/bbauvsbc1/bbauv/src/bbsonar/grayIntensities.txt" ;
	const std::string COLOR_IMAGE_FILE = "/home/bbauvsbc1/bbauv/src/bbsonar/colorImage.png" ;
    const std::string LABELLED_IMAGE_FILE = "/home/bbauvsbc1/bbauv/src/bbsonar/labelledImg.png" ;

#endif

// grayscale image constants
#define IMAGE_TYPE	BVTHEAD_IMAGE_XY		// option(s): XY, RTHETA
#define RES_TYPE	BVTHEAD_RES_AUTO		// option(s): OFF, LOW, MED, HIGH, AUTO

// file path related constants
//const std::string SONAR_LOG_PATH = "sonarLog.son" ;
const std::string SONAR_STATIC_FILE = "/home/bbauvsbc1/ash_store/bvtsdk/data/swimmer.son" ;
const std::string COLOR_MAPPER_PATH = "/home/bbauvsbc1/bbauv/src/bbsonar/colormaps/cool.cmap" ;

// range related constants
const int MIN_RANGE = 2;
const int MAX_RANGE = 13;
const int GRAYSCALE_THRESH = 150;

// for global thresholding (adding a constant)
const int THRESH_CONSTANT = 100;
const int ROWS_CROPPED = 50;

// for gamma correction, optimal values
const float PL_CONST = 0.1;
const float PL_GAMMA = 1.43;

// for contour filtering
const double CONTOUR_AREA_LOWER_BOUND = 15.0;
const double CONTOUR_AREA_UPPER_BOUND = 100.0;

// miscellaneous constants
//const int HOTSPOT_THRESH = 150;

#endif /* CONFIG_H_ */

