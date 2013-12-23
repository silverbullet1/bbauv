#include "pipe_task.h"

namespace bbauv_vision{
  
	pipe_task::pipe_task():
		private_nh_("~"),
		it_(nh_)
  {
		cam_sub_ = it_.subscribeCamera("camera/image_rect_color", 1, &pipe_task::ImageCB, this);

		find_times_ = 0;
		find_times_limit_ = 30;
		cameraInfo_initialized_ = false;

		pipe_length_total_ = 0.305;

		pipePose_pub_ = nh_.advertise<bbauv_vision_tasks::pipe_pose >("/pipe_pose", 1);
  }

  
  void pipe_task::ImageCB( const sensor_msgs::ImageConstPtr& image_msg,
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

		Mat hsv_image;
		cvtColor( cv_ptr->image, hsv_image, CV_BGR2HSV);
		//http://stackoverflow.com/questions/10948589/choosing-correct-hsv-values-for-opencv-thresholding-with-inranges
		Scalar color_min = Scalar(20, 10, 20);
		Scalar color_max = Scalar(90, 50, 100);

		Mat pipe_orange_threshold;
		inRange(hsv_image, color_min, color_max, pipe_orange_threshold);
		//ROS_INFO("inrange dst channel number %d", pipe_orange_threshold.channels());

		int opening_size = 1;
		Mat opening_element = getStructuringElement( MORPH_RECT, Size( 2*opening_size + 1, 2*opening_size+1 ));
		int closing_size = 5;
		Mat closing_element = getStructuringElement( MORPH_RECT, Size( 2*closing_size + 1, 2*closing_size+1 ));

		morphologyEx(pipe_orange_threshold, pipe_orange_threshold, MORPH_OPEN, opening_element);
		morphologyEx(pipe_orange_threshold, pipe_orange_threshold, MORPH_CLOSE, closing_element);
		imshow("raw_threshold",pipe_orange_threshold);

		int white_pixel_num = countNonZero(pipe_orange_threshold);

		Mat copy_for_contour = pipe_orange_threshold.clone();

		vector<vector<cv::Point> > contours;
		findContours( copy_for_contour, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
		for(size_t i=1; i<contours.size(); i++)
		{
			line(pipe_orange_threshold, contours[i-1].front(),contours[i].front(), Scalar(255), 2);
		}
		imshow("pipe_outer",pipe_orange_threshold);

		Mat copy_for_contour2 = pipe_orange_threshold.clone();
		vector<vector<cv::Point> > single_contour;
		findContours( copy_for_contour2, single_contour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

		bbauv_vision_tasks::pipe_pose pipe_skeleton_pose;
		pipe_skeleton_pose.header = cv_ptr->header;

		if(white_pixel_num>200)
		{
			RotatedRect minArea_rectangle;
			minArea_rectangle = minAreaRect( Mat(single_contour[0]) );
			Mat color_rect;
			cvtColor( pipe_orange_threshold, color_rect, CV_GRAY2BGR);

			Point2f rect_points[4];
			minArea_rectangle.points( rect_points );
			for( int j = 0; j < 4; j++ )
			{
				line( color_rect, rect_points[j], rect_points[(j+1)%4], Scalar(255, 0, 0), 3, 8 );
			}

			Point2f lowest_pt, second_lowest_pt;
			pipe_skeleton_pose.detect_pipe  = Calc_pose(rect_points, lowest_pt, second_lowest_pt, pipe_skeleton_pose);
			line( color_rect, lowest_pt, second_lowest_pt, Scalar(0, 0, 255), 3, 8 );

			imshow("color_rect",color_rect);
		}

		if(pipe_skeleton_pose.detect_pipe){find_times_++;}
		else {find_times_ --;}

		if(find_times_ > find_times_limit_)	pipe_skeleton_pose.detection_stable = true;
		else pipe_skeleton_pose.detection_stable = false;

		pipePose_pub_.publish(pipe_skeleton_pose);

		imshow("orange_theshold",pipe_orange_threshold);
		imshow("pipe_outer",pipe_orange_threshold);
		waitKey(2);
  }


  bool pipe_task::Calc_pose(Point2f *rect_points, Point2f& lowest_pt, Point2f& second_lowest_pt, bbauv_vision_tasks::pipe_pose & pipeFrame_pose)
  {
	  double dist1 = sqrtf((rect_points[0].x - rect_points[1].x)*(rect_points[0].x - rect_points[1].x)+(rect_points[0].y - rect_points[1].y)*(rect_points[0].y - rect_points[1].y));
	  double dist2 = sqrtf((rect_points[2].x - rect_points[1].x)*(rect_points[2].x - rect_points[1].x)+(rect_points[0].y - rect_points[1].y)*(rect_points[0].y - rect_points[1].y));
	  double longer_line = dist1>=dist2?dist1:dist2;
	  double shorter_line = dist1<dist2?dist1:dist2;
	  if(shorter_line < 0.001) return false;
	  if(longer_line/shorter_line >3.0) return false;

	  int points_border_image = 0;
	  for(int i=0; i<4; i++)
	  {
		  if(rect_points[i].x<2.0 || rect_points[i].y<2.0) points_border_image++;
	  }
	  if(points_border_image>=3) return false;

	  //find the lowest 2 points in the image, to serve as the two ends of the lowest pipeline bar;
	  Point2f lowest_point, second_lowest_point;
	  if(rect_points[0].y>rect_points[1].y){lowest_point = rect_points[0]; second_lowest_point = rect_points[1];}
	  else{lowest_point = rect_points[1]; second_lowest_point = rect_points[0];}
	  if(rect_points[2].y>lowest_point.y){second_lowest_point = lowest_point; lowest_point = rect_points [2];}
	  else if(rect_points[2].y>second_lowest_point.y){second_lowest_point = rect_points[2];}
	  if(rect_points[3].y>lowest_point.y){second_lowest_point = lowest_point; lowest_point = rect_points [3];}
	  else if(rect_points[3].y>second_lowest_point.y){second_lowest_point = rect_points[3];}

	  lowest_pt = lowest_point;
	  second_lowest_pt = second_lowest_point;

	  double length_pixel_ratio = pipe_length_total_ / longer_line;
	  double estimated_depth = length_pixel_ratio * camera_info_.K[0];

	  geometry_msgs::Point32 realPt1, realPt2, tmpPoint;
	  imagePt_to_realPt(lowest_point, realPt1, length_pixel_ratio, estimated_depth);
	  imagePt_to_realPt(second_lowest_point, realPt2, length_pixel_ratio, estimated_depth);
	  if(realPt1.y>realPt2.y){tmpPoint = realPt1; realPt1=realPt2; realPt2 = tmpPoint;}

	  pipeFrame_pose.points.push_back(realPt1);
	  pipeFrame_pose.points.push_back(realPt2);

	  return true;
  }

  void pipe_task::imagePt_to_realPt(Point2f &imagePt, geometry_msgs::Point32 &realPt, double ratio, double depth_z)
  {
	  realPt.x = ratio*(imagePt.x - camera_info_.K[2]);
	  realPt.y = ratio*(imagePt.y - camera_info_.K[5]);
	  realPt.z = depth_z;
  }

  pipe_task::~pipe_task(){ }

};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "pipe_task");
	 ros::NodeHandle n;
	 bbauv_vision::pipe_task pipe_task_node;
     ros::spin();
     return 0;
}
