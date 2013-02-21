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

Mat src, src_gray, bw, bw2, frame;

// Configurable values
int GRAY_LOW=0, GRAY_HIGH=40;
int HOR_PIXEL_LIMIT = 55;
float FORWARD_SETPOINT = 1.2;

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
	createTrackbar( "Gray low: ", "src", &GRAY_LOW, 255, 0);
	createTrackbar( "Gray high: ", "src", &GRAY_HIGH, 255, 0);
	createTrackbar( "Hor pixel limit: ", "src", &HOR_PIXEL_LIMIT, 255, 0);
	createTrackbar( "Forward setpoint: ", "src", &FORWARD_SETPOINT, 255, 0);

	//VideoCapture cap(0);

	while(true) {
		//cap >> src;
		src = frame;

		int c = waitKey( 20 );
		if( c == 27 ) break;

		cvtColor( src, src_gray, CV_BGR2GRAY );
		imshow("src_gray", src_gray);

//		minAreaRect(contours);
		inRange(src_gray, GRAY_LOW, GRAY_HIGH, bw);
		imshow("bw", bw);

		bw2 = bw;

		vector<vector<Point> > contours;
		findContours(bw2, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		vector<RotatedRect> boundRect( contours.size() );
		//drawContours(src, contours, -1, Scalar(255), 2);
		for( int i = 0; i < contours.size(); i++ )
		{
			boundRect[i] = minAreaRect( contours[i] );
		}

		/// Draw bounding rectangles
		Mat drawing = Mat::zeros( src_gray.size(), CV_8UC3 );
		int centerX = src_gray.size().width/2;
		int centerY = src_gray.size().height/2;

		int count = 0;
		for( int i = 0; i< contours.size(); i++ ){
			if(contourArea(contours[i]) > 1000){
				Scalar color = Scalar( 255 );
				Point2f rect_points[4]; boundRect[i].points( rect_points );
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


				///Display the Count, Skew Angle, Centroid x and y
				double dist = norm(boundRect[i].center - Point2f(centerX, centerY));
				Point2f delta = boundRect[i].center - Point2f(centerX, centerY);
				printf("%i: %f diff_angle: %f c: %f %f  %lf\n", count++, boundRect[i].angle, diff_angle, boundRect[i].center.x, boundRect[i].center.y, dist);


				//FIXME: Temporarily use geometry msg: x = forward, y = sideway, z = rotational velocities
				bbauv_msgs::controller_input sendmsg;

				sendmsg.depth_setpoint = 0.8;
				sendmsg.heading_setpoint = heading;

				if (delta.x < -HOR_PIXEL_LIMIT) {
					sendmsg.heading_setpoint = heading - 10;
					sendmsg.forward_setpoint = FORWARD_SETPOINT;
					sendmsg.sidemove_setpoint = 0.15;

					printf("deltax: %f\n", delta.x);
				}
				else if (delta.x > HOR_PIXEL_LIMIT) {
					sendmsg.heading_setpoint = heading + 10;
					sendmsg.forward_setpoint = FORWARD_SETPOINT;
					sendmsg.sidemove_setpoint = -0.15;

					printf("deltax: %f\n", delta.x);
				}
				 else {
					if (abs(diff_angle) > 10) {
						printf("rotating from %f to %f\n", heading, heading - diff_angle);
						sendmsg.heading_setpoint = heading - diff_angle;
						sendmsg.forward_setpoint = 0.4;
					} else {
						sendmsg.forward_setpoint = FORWARD_SETPOINT;
					}
				}

				movement_pub.publish(sendmsg);
/*
				if(delta.x < -threshold)
				{
					if(boundRect[i].angle < 0)
					{
						if(boundRect[i].angle < -45)
						{
							printf("spin left until angle = -10\n");
							sendmsg.heading_setpoint = heading + diff_angle;
							movement_pub.publish(sendmsg);
						}
						else
						{
							printf("spin right until angle = 10\n");
							sendmsg.heading_setpoint = heading + diff_angle;
							movement_pub.publish(sendmsg);
						}
					}
					else
					{
						printf("forward\n");
						sendmsg.heading_setpoint = heading;
//						sendmsg.forward_setpoint = 0.2;
						movement_pub.publish(sendmsg);
					}
				}
				else if(delta.x > threshold)
				{
					printf("forward\n");
					sendmsg.heading_setpoint = heading;
//					sendmsg.forward_setpoint = 0.2;
					movement_pub.publish(sendmsg);
				}
				else
				{
					printf("forward\n");
					sendmsg.heading_setpoint = heading;
//					sendmsg.forward_setpoint = 0.2;
					movement_pub.publish(sendmsg);
				}
*/
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
