#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <bbauv_msgs/controller_input.h>
#include <bbauv_msgs/compass_data.h>
#include <cv_bridge/cv_bridge.h>
#include <stdlib.h>
#include <stdio.h>

using namespace cv;
using namespace std;

Mat src, src_gray, bw, bw2, frame, element3(3,3,CV_8U,cv::Scalar(1));
					
int low, high=40, centerX, centerY;
float heading;

image_transport::Subscriber image_sub;
ros::Subscriber compass_sub;

ros::Publisher movement_pub;

void compass_callback(const bbauv_msgs::compass_dataConstPtr& message) {
	heading = message->yaw;
}

void image_callback(const sensor_msgs::ImageConstPtr& message) {
	frame = cv_bridge::toCvCopy(message)->image;
}

void init()
{
	// Get some private params at the start
	string topic_name, video_name;
	int webcam_id;
	ros::NodeHandle private_nh("~");
	private_nh.param<string>("topic_name", topic_name, "/bottomcam/camera/image_raw");
	private_nh.param<string>("video_name", video_name, "");
	private_nh.param<int>("webcam_id", webcam_id, 0);

/*	
	if (topic_name.empty()) {
		ROS_INFO("No topic name specified, defaulting to video or own webcam.");

		if (video_name.empty()) {
			ROS_INFO("No video name specified, defaulting to own webcam.");
			taker=new Img_Taker(webcam_id);
		} else {
			ROS_INFO("Playing video %s.", video_name.c_str());
			taker=new Img_Taker(video_name);
		}
	} else {
		ROS_INFO("Subscribing to topic %s.", topic_name.c_str());
		taker = new Img_Subscriber(topic_name);
	}
*/
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_sub = it.subscribe(topic_name, 1, &image_callback);

	compass_sub = n.subscribe("/os5000_data", 1000, &compass_callback);

	movement_pub = n.advertise<bbauv_msgs::controller_input>("/line_follower", 1000);

	frame = Mat::zeros(240, 320, CV_8UC3);
}


int main( int argc, char** argv )
{
	ros::init(argc, argv, "line_follower");
	init();

	namedWindow("src", CV_WINDOW_AUTOSIZE);
	createTrackbar( "Low: ", "src", &low, 255, 0);
	createTrackbar( "High: ", "src", &high, 255, 0);

	double reverseTill = 0;
	bool reversed = false;

	double waitTill = 0; // Quick hack to wait
	//double curTime = ros::Time::now().toSec();
	while(true) {
		double curTime = ros::Time::now().toSec();

		src = frame;

		int c = waitKey( 20 );
		if( c == 27 ) break;

		cvtColor( src, src_gray, CV_BGR2GRAY );
		imshow("src_gray", src_gray);

		inRange(src_gray, low, high, bw);
		//imshow("bw", bw);
		//erode(bw, bw, element3);
		//dilate(bw, bw, element3);
		imshow("bw", bw);

		vector<vector<Point> > contours;
		findContours(bw, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		vector<RotatedRect> boundRect( contours.size() );
		for( int i = 0; i < contours.size(); i++ )
		{
			boundRect[i] = minAreaRect( contours[i] );
		}

		/// Draw bounding rectangles
		Mat drawing = Mat::zeros( src_gray.size(), CV_8UC3 );
		centerX = src_gray.size().width/2;
		centerY = src_gray.size().height/2;

		// Find the largest contour
		int count = 0;
		int maxIndex = 0;
		float maxArea = 0;
		for( int i = 0; i< contours.size(); i++ ){
			float curArea = contourArea(contours[i]);
			if (curArea > 300 && curArea > maxArea) {
				maxArea = curArea;
				maxIndex = i;
				count++;
			}
		}

		if (curTime >= waitTill) {
			if (count) {
				RotatedRect &largestRect = boundRect[maxIndex];
				Scalar color = Scalar( 255 );
				Point2f rect_points[4]; largestRect.points( rect_points );
				for(int j = 0; j < 4; j++)
				{
					line(drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
				}

				Point2f edges[2] = { rect_points[1] - rect_points[0],
					rect_points[2] - rect_points[1] };

				float diff_angle = 0;
				if (norm(edges[0]) > norm(edges[1])) {
					diff_angle = atan(edges[0].x / edges[0].y) / M_PI * 180.;
				} else {
					diff_angle = atan(edges[1].x / edges[1].y) / M_PI * 180.;
				}

				bool below_mid = (rect_points[0].y > centerY) && (rect_points[1].y > centerY) && (rect_points[2].y > centerY) && (rect_points[3].y > centerY);

				///Display the Count, Skew Angle, Centroid x and y
				double dist = norm(largestRect.center - Point2f(centerX, centerY));
				Point2f delta = largestRect.center - Point2f(centerX, centerY);
				printf("%i: %f diff_angle: %f c: %f %f  %lf\n", count++, largestRect.angle, diff_angle, largestRect.center.x, largestRect.center.y, dist);


				const float FORWARD_SETPOINT = 0.4;
				bbauv_msgs::controller_input sendmsg;

				float x_ratio, y_ratio;
				x_ratio = float(largestRect.center.x) / src_gray.size().width;
				y_ratio = float(largestRect.center.y) / src_gray.size().height;

				sendmsg.depth_setpoint = 0.2;
				sendmsg.heading_setpoint = heading;

				float factor = 0.5f + (centerY - largestRect.center.y) / float(src_gray.size().height);

				double threshold = 50.0;
				if (x_ratio > 0.5) {
					if (reversed && reverseTill < curTime) {
						printf("sharp turn??? rotate to: %f\n", sendmsg.heading_setpoint);
						
						sendmsg.heading_setpoint = heading + 90;
						sendmsg.sidemove_setpoint = -0.4;

						waitTill = ros::Time::now().toSec() + 3;
						reversed = false;
					} else if (!reversed) {
						sendmsg.forward_setpoint = -0.8;
						movement_pub.publish(sendmsg);

						reversed = true;
						reverseTill = ros::Time::now().toSec() + 1.5;
					}
				}
				//TODO: do the other case (like the above, but left)
				else if (x_ratio < 0.5 && reverseTill < curTime) {
					if (reversed && reverseTill < curTime) {
						printf("sharp turn??? rotate to: %f\n", sendmsg.heading_setpoint);

						sendmsg.heading_setpoint = heading - 90;
						sendmsg.sidemove_setpoint = 0.4;

						waitTill = ros::Time::now().toSec() + 3;
						reversed = false;
					} else if (!reversed) {
						sendmsg.forward_setpoint = -0.8;
						movement_pub.publish(sendmsg);

						reversed = true;
						reverseTill = ros::Time::now().toSec() + 1.5;
					}
				}
				//TODO: do the other case (like the above, but left)
				else if (delta.x < -threshold) {
					sendmsg.heading_setpoint = heading - 10;
					sendmsg.forward_setpoint = factor * FORWARD_SETPOINT;
					sendmsg.sidemove_setpoint = 0.2;

					printf("deltax: %f\n", delta.x);
					printf("factor: %f\n", factor);
				}
				else if (delta.x > threshold) {
					sendmsg.heading_setpoint = heading + 10;
					sendmsg.forward_setpoint = factor * FORWARD_SETPOINT;
					sendmsg.sidemove_setpoint = -0.2;

					printf("deltax: %f\n", delta.x);
					printf("factor: %f\n", factor);
				}
				else {
					if (abs(diff_angle) > 10) {
						printf("rotating from %f to %f\n", heading, heading - diff_angle);
						sendmsg.heading_setpoint = heading - diff_angle;
						sendmsg.forward_setpoint = factor * 0.4;
						printf("factor: %f\n", factor);
					} else {
						sendmsg.forward_setpoint = factor * FORWARD_SETPOINT;
						printf("factor: %f\n", factor);
					}
				}

				movement_pub.publish(sendmsg);
			}
			else { // Count == 0
				printf("LOST LINE. Stopping.\n");
				bbauv_msgs::controller_input sendmsg;
				sendmsg.heading_setpoint = heading;
				sendmsg.depth_setpoint = 0.2;
				movement_pub.publish(sendmsg);
			}
		}

		/// Show in a window
		namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
		imshow( "Contours", drawing );


		imshow("src", src);
		

		ros::spinOnce();
	}

	return 0;
}
