/*
 * flarestates.cpp
 *
 *  Created on: 22 Jan, 2014
 *      Author: lynnette & thien
 */

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <string.h>

#include "flarestates.h"

//Look for flare state
LookForFlareState::LookForFlareState(){
	ROS_INFO("Looking for yellow flare");
}

boost::shared_ptr<State> LookForFlareState::gotFrame(cv::Mat image, RectData rectData)P
		return shared_from_this();
}

//Lost flare - should keep turning around
LostFlareState::LostFlareState(){
	ROS_INFO("Lost the flare...");
}

boost::shared_ptr<State> LostFlareState::gotFrame(cv::Mat image, RectData rectData){
		return shared_from_this();
}


