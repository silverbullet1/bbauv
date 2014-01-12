//States for the linefollower

/* 
	linefollowerstates.cpp
	States for line follower
	Date created: 10 Jan 2014
	Author: Lynnette
*/

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string.h>

#include "linefollowingstates.h"

// Abstract base state
void State::setInputImage(Mat image) {
	this->inImage = image;
}

// Look for line state

