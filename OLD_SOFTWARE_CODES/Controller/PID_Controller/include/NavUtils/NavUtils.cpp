/*
 * NavUtils.cpp
 *
 *  Created on: May 20, 2013
 *      Author: gohew
 */

#include "NavUtils.h"
#include<sensor_msgs/Imu.h>


NavUtils::NavUtils() {
	// TODO Auto-generated constructor stub

}

double NavUtils::quaternionToPitch(double q0, double q1, double q2, double q3)
{
	double p = asin(2 * (q0*q2 - q3*q1));
	p = p * 180.0 / M_PI;
	return p;
}

double NavUtils::quaternionToRoll(double q0, double q1, double q2, double q3)
{
	double r = atan2(2 * (q0*q1 + q2*q3), 1-2*(q1*q1 + q2*q2));
	r = r * 180.0 / M_PI;
	return r;
}

double NavUtils::quaternionToYaw(double q0, double q1, double q2, double q3)
{
	double y = atan2(2 * (q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3));
	y = y * 180.0 / M_PI;
	return y;
}

NavUtils::~NavUtils() {
	// TODO Auto-generated destructor stub
}

