/*
 * NavUtils.h
 *
 *  Created on: May 20, 2013
 *      Author: gohew
 */

#ifndef NAVUTILS_H_
#define NAVUTILS_H_

#include <ros/ros.h>

class NavUtils {
public:
	NavUtils();
	double quaternionToPitch(double q0, double q1, double q2, double q3);
	double quaternionToYaw(double q0, double q1, double q2, double q3);
	double quaternionToRoll(double q0, double q1, double q2, double q3);

	virtual ~NavUtils();
};

#endif /* NAVUTILS_H_ */
