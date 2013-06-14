#ifndef BBAUV_PCL_PROCESS_H
#define BBAUV_PCL_PROCESS_H

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include "pcl/point_cloud.h"
#include "pcl_ros/point_cloud.h"
//#include <fmutil/fm_math.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include "stereo_vision_bbauv/roiDist.h"
#include <sensor_msgs/CameraInfo.h>

using namespace std;

namespace bbauv_pcl{

	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

	class pcl_process {
        public:
		pcl_process();
        ~pcl_process();
    
        private:
        ros::NodeHandle nh_, private_nh_;
        string odom_frame_, base_frame_;
        tf::TransformListener *tf_;
        PointCloudRGB latest_cloud_;

        bool process_flag_;
        int  times_;

        ros::Subscriber                             cameraInfo_sub_;
		ros::Subscriber                             pcl_sub_;
		ros::Publisher                              laser_pub_;
		ros::Publisher                              pcl_vis_pub_;

		boost::recursive_mutex 						configuration_mutex_;

		bool cameraInfo_initilized_;
		double focal_length_x_, focal_length_y_;
		double centroid_x_, centroid_y_;

		double z_limit_;
		ros::ServiceServer roiDist_service_;

		void pclCallback(const PointCloudRGB::ConstPtr& pcl_in);
		void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info);
		bool ROI_DistCalc(stereo_vision_bbauv::roiDist::Request  &req, stereo_vision_bbauv::roiDist::Response &res);

		void plane_fitting(const PointCloudRGB::ConstPtr& pcl_in);
		int plane_classication(pcl::PointIndices &plane_indices, pcl::ModelCoefficients &plane_coefs, PointCloudRGB &pcl_all);
		void synthetic_LIDAR();
		void check_process();
    };

};

#endif

