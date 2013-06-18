#ifndef BBAUV_VISION_TASK_PIPE_H
#define BBAUV_VISION_TASK_PIPE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <cstdio>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include "bbauv_vision_tasks/pipe_pose.h"

using namespace std;
using namespace ros;
using namespace cv;

namespace bbauv_vision{

    class pipe_task {
        public:
    	pipe_task();
        ~pipe_task();
    
        private:
        ros::NodeHandle nh_, private_nh_;
        image_transport::ImageTransport it_;
        image_transport::CameraSubscriber cam_sub_;
        image_transport::Publisher binary_pub_;
        sensor_msgs::CameraInfo camera_info_;

        ros::Publisher pipePose_pub_;

        bool cameraInfo_initialized_;
        double pipe_length_total_;
        double pipe_length_little_;
        int find_times_;
        int find_times_limit_;

        void ImageCB(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
        bool Calc_pose(Point2f *rect_points, Point2f& lowest_pt, Point2f& second_lowest_pt, bbauv_vision_tasks::pipe_pose & pipeFrame_pose);
        void imagePt_to_realPt(Point2f &imagePt, geometry_msgs::Point32 &realPt, double ratio, double depth_z);
   };
};

#endif
