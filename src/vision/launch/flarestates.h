/*
 * flarestates.h
 *
 *  Created on: 22 Jan, 2014
 *      Author: lynnette & thien
 */

#ifndef FLARESTATES_H_
#define FLARESTATES_H_

#include <ros/ros.h>
#include <bbauv_msgs/controller.h>

#include <boost/enable_shared_from_this.hpp>

#include <stdlib.h>
#include <string.h>

//Global constants
extern ros::Publisher movementPub;

inline void publishMovement(const bbauv_msgs::controller& movement){
	movementPub.publish(movement);
}

//Structure for bounding box
struct RectData{
	bool detected;
	cv::Point2f center;
	cv::RotatedRect maxRect;
};

//Abstract base class for states
class State : public boost::enable_shared_from_this<State>{
public:
	State() {};
	virtual ~State() {};
	virtual boost::shared_ptr<State> gotFrame(cv::Mat, RectData rectData) = 0;
};

// Looking for a flare
class LookForFlareState : public State{
public:
	LookForFlare();
	boost::shared_ptr<State> gotFrame(cv::Mat, RectData);
};

// Lost flare
class LostFlareState : public State{
public:
	LostFlare();
	boost::shared_ptr<State> gotFrame(cv::Mat, RectData);
};

#endif /* FLARESTATES_H_ */
