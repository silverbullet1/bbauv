#include "wheel_task.h"

namespace bbauv_vision{
  
	wheel_task::wheel_task():
		private_nh_("~"),
		it_(nh_)
  {
		cam_sub_ = it_.subscribeCamera("/stereo_camera/left/image_rect_color", 1, &wheel_task::ImageCB, this);

		find_times_ = 0;
		find_times_limit_ = 30;
		cameraInfo_initialized_ = false;

		wheel_diameter_ = 0.38;
		meter_pixel_ratio_ = 0.0;
		estimated_depth_ = 0.0;

		pipePose_pub_ = nh_.advertise<bbauv_vision_tasks::wheel_task_pose >("/wheel_task_pose", 1);
  }

  
  void wheel_task::ImageCB( const sensor_msgs::ImageConstPtr& image_msg,
                           const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
        ROS_INFO("pipe_task----ImageCB-----");

        if(!cameraInfo_initialized_)
        {
        	camera_info_ = *info_msg;
        	cameraInfo_initialized_ = true;
        }

		cv_bridge::CvImagePtr cv_ptr;
		try
		{
		  cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
		  ROS_ERROR("cv_bridge exception: %s", e.what());
		  return;
		}

		Mat hsv_image, hsv_clone1, hsv_clone2;
		cvtColor( cv_ptr->image, hsv_image, CV_BGR2HSV);
		hsv_clone1 = hsv_image.clone();
		hsv_clone2 = hsv_image.clone();

		//http://stackoverflow.com/questions/10948589/choosing-correct-hsv-values-for-opencv-thresholding-with-inranges
		Mat yellow_image, red_image, green_image;

		yellow_threshold(hsv_image, yellow_image);
		red_threshold(hsv_clone1,red_image);
		green_threshold(hsv_clone2, green_image);

		bbauv_vision_tasks::wheel_task_pose task_pose;
		task_pose.header = cv_ptr->header;
		yellow_board_detection(yellow_image, task_pose);
		red_wheel_detection(red_image, task_pose);
		green_bar_detection(green_image, task_pose);

		pipePose_pub_.publish(task_pose);

		imshow("green_image",green_image);
		imshow("red_image",red_image);
		imshow("yellow_image",yellow_image);
		waitKey(2);
  }

	void wheel_task::yellow_threshold(Mat &input_image, Mat &yellow_image)
	{
		Scalar color_min = Scalar(15, 100, 120);
		Scalar color_max = Scalar(60, 255, 255);

		inRange(input_image, color_min, color_max, yellow_image);

		int opening_size = 1;
		Mat opening_element = getStructuringElement( MORPH_RECT, Size( 2*opening_size + 1, 2*opening_size+1 ));
		int closing_size = 3;
		Mat closing_element = getStructuringElement( MORPH_RECT, Size( 2*closing_size + 1, 2*closing_size+1 ));

		morphologyEx(yellow_image, yellow_image, MORPH_OPEN, opening_element);
		morphologyEx(yellow_image, yellow_image, MORPH_CLOSE, closing_element);
	}

  void wheel_task::red_threshold(Mat &input_image, Mat &red_image)
  {
		Scalar color_min = Scalar(1, 180, 80);
		Scalar color_max = Scalar(10, 255, 255);

		inRange(input_image, color_min, color_max, red_image);

		int opening_size = 1;
		Mat opening_element = getStructuringElement( MORPH_RECT, Size( 2*opening_size + 1, 2*opening_size+1 ));
		int closing_size = 3;
		Mat closing_element = getStructuringElement( MORPH_RECT, Size( 2*closing_size + 1, 2*closing_size+1 ));

		morphologyEx(red_image, red_image, MORPH_OPEN, opening_element);
		morphologyEx(red_image, red_image, MORPH_CLOSE, closing_element);
  }

  void wheel_task::green_threshold(Mat &input_image, Mat &green_image)
  {
		Scalar color_min = Scalar(40, 140, 160);
		Scalar color_max = Scalar(60, 200, 255);

		inRange(input_image, color_min, color_max, green_image);

		int opening_size = 1;
		Mat opening_element = getStructuringElement( MORPH_RECT, Size( 2*opening_size + 1, 2*opening_size+1 ));
		int closing_size = 3;
		Mat closing_element = getStructuringElement( MORPH_RECT, Size( 2*closing_size + 1, 2*closing_size+1 ));

		morphologyEx(green_image, green_image, MORPH_OPEN, opening_element);
		morphologyEx(green_image, green_image, MORPH_CLOSE, closing_element);
  }

	void wheel_task::yellow_board_detection(Mat &yellow_image, 	bbauv_vision_tasks::wheel_task_pose &wheelTaskPose)
	{
		Mat image_copy = yellow_image.clone();
		vector<vector<cv::Point> > contours;
		findContours( image_copy, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

		vector<vector<Point> > contours_poly( contours.size() );
		vector<RotatedRect> boundRotateRect( contours.size() );

		for( size_t i = 0; i < contours.size(); i++ )
		{
			approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
			boundRotateRect[i] = minAreaRect( Mat(contours_poly[i]) );
		}

		float max_area = 0.0;
		int maxArea_serial = 0;
		for( size_t i = 0; i < boundRotateRect.size(); i++ )
		{
			float rect_area = boundRotateRect[i].size.height * boundRotateRect[i].size.width;
			max_area = max_area>rect_area ? max_area: rect_area;
			maxArea_serial = max_area>rect_area ? maxArea_serial:i;
		}

		if(max_area > 60) wheelTaskPose.detect_yellow_board = true;
	}

  void wheel_task::red_wheel_detection(	Mat &red_image, bbauv_vision_tasks::wheel_task_pose &wheelTaskPose)
  {
	  Mat image_copy = red_image.clone();
	vector<vector<cv::Point> > contours;
	findContours( image_copy, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

	vector<vector<Point> > contours_poly( contours.size() );
	vector<Point2f>center( contours.size() );
	vector<float>radius( contours.size() );

	for( size_t i = 0; i < contours.size(); i++ )
	{
		approxPolyDP( Mat(contours[i]), contours_poly[i], 1, true );
		minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i]);
	}

	float max_radius = 0.0;
	int maxRadius_serial = 0;
	for( size_t i = 0; i < radius.size(); i++ )
	{
		max_radius = max_radius > radius[i] ? max_radius: radius[i];
		maxRadius_serial = max_radius > radius[i]? maxRadius_serial:i;
	}

	if(max_radius > 10)
	{
		wheelTaskPose.detect_red_wheel = true;
		double diameter = 2.0 * radius[maxRadius_serial];
		meter_pixel_ratio_ = wheel_diameter_ / diameter;
	    estimated_depth_ = meter_pixel_ratio_ * camera_info_.K[0];

		geometry_msgs::Point32 wheel_center;
		imagePt_to_realPt(center[maxRadius_serial], wheel_center, meter_pixel_ratio_, estimated_depth_);
		wheelTaskPose.wheel_center = wheel_center;
	}
	else
	{
		wheelTaskPose.detect_red_wheel = false;
		meter_pixel_ratio_ = 0.0;
		estimated_depth_ = 0.0;
	}

  }

  void wheel_task::green_bar_detection(	Mat &green_image, 	bbauv_vision_tasks::wheel_task_pose &wheelTaskPose)
  {
	    Mat image_copy = green_image.clone();
		vector<vector<cv::Point> > contours;
		findContours( image_copy, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

		vector<vector<Point> > contours_poly( contours.size() );
		vector<RotatedRect> boundRotateRect( contours.size() );

		for( size_t i = 0; i < contours.size(); i++ )
		{
			approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
			boundRotateRect[i] = minAreaRect( Mat(contours_poly[i]) );
		}

		float max_area = 0.0;
		int maxArea_serial = 0;
		for( size_t i = 0; i < boundRotateRect.size(); i++ )
		{
			float rect_area = boundRotateRect[i].size.height * boundRotateRect[i].size.width;
			max_area = max_area>rect_area ? max_area: rect_area;
			maxArea_serial = max_area>rect_area ? maxArea_serial:i;
		}

		if(max_area > 10.0)
		{
			wheelTaskPose.detect_green_bar = true;
			geometry_msgs::Point32 bar_center;
			if(wheelTaskPose.detect_red_wheel)
			{
				imagePt_to_realPt(boundRotateRect[maxArea_serial].center, bar_center, meter_pixel_ratio_, estimated_depth_);
				wheelTaskPose.bar_angle = atan2(bar_center.y-wheelTaskPose.wheel_center.y, bar_center.x-wheelTaskPose.wheel_center.x)*180.0/M_PI;
			}
		}
  }

  //pose in the OpenCV convention camera-coordinate, pay attention to change into ROS coordinate;
  void wheel_task::imagePt_to_realPt(Point2f &imagePt, geometry_msgs::Point32 &realPt, double ratio, double depth_z)
  {
	  realPt.x = ratio*(imagePt.x - camera_info_.K[2]);
	  realPt.y = ratio*(imagePt.y - camera_info_.K[5]);
	  realPt.z = depth_z;
  }

  wheel_task::~wheel_task(){ }

};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "wheel_task");
	 ros::NodeHandle n;
	 bbauv_vision::wheel_task wheel_task_node;
     ros::spin();
     return 0;
}
