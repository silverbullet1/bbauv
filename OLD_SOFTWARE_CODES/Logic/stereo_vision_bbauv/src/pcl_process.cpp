#include "pcl_process.h"

namespace bbauv_pcl{
	
	pcl_process::pcl_process():
    private_nh_("~")
    {
		odom_frame_ = "odom";
		base_frame_ = "base_link";
		times_ = 0;
		cameraInfo_initilized_ = false;

		tf_ = new tf::TransformListener();
		pcl_sub_ = nh_.subscribe("/cloud", 100, &pcl_process::pclCallback, this);
		cameraInfo_sub_ = nh_.subscribe("/stereo_camera/right/camera_info", 100, &pcl_process::cameraInfoCallback, this);
		roiDist_service_ = nh_.advertiseService("/calcRoiDist", &pcl_process::ROI_DistCalc, this);

		private_nh_.param("z_limit",  			z_limit_,			5.0);

		pcl_vis_pub_   = nh_.advertise<PointCloud>("/pcl_vis_plane", 10);
	}
	
	pcl_process::~pcl_process()
	{
		delete tf_;
	}

	void pcl_process::pclCallback(const PointCloudRGB::ConstPtr& pcl_in)
	{
		boost::recursive_mutex::scoped_lock pclCB(configuration_mutex_);
		latest_cloud_ = *pcl_in;
		//plane_fitting(pcl_in);
	}

	void pcl_process::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info)
	{
		if(!cameraInfo_initilized_)
		{
			focal_length_x_ = camera_info->K[0];
			focal_length_y_ = camera_info->K[4];
			centroid_x_ = camera_info->K[2];
			centroid_y_ = camera_info->K[5];
			cameraInfo_initilized_ = true;
		}
	}

	bool pcl_process::ROI_DistCalc(stereo_vision_bbauv::roiDist::Request  &req, stereo_vision_bbauv::roiDist::Response &res)
	{
		ROS_INFO("ROI_DistCalc service called");
		res.roi_dist = 0.0;
		if(!cameraInfo_initilized_)
		{
			ROS_WARN("camera_info hasn't been initilized, please check the topic");
			return true;
		}

		double angle_limit_X_lower, angle_limit_X_upper;
		double angle_limit_Y_lower, angle_limit_Y_upper;
		angle_limit_X_lower = atan2(focal_length_x_, double(req.topleft_x+req.rect_width-centroid_x_));
		angle_limit_X_upper = atan2(focal_length_x_, double(req.topleft_x-centroid_x_));
		angle_limit_Y_lower = atan2(focal_length_y_, double(req.topleft_y+req.rect_height-centroid_y_));
		angle_limit_Y_upper = atan2(focal_length_y_, double(req.topleft_y-centroid_y_));

		ROS_INFO("angle range: x (%f, %f); y (%f, %f)", angle_limit_X_lower,  angle_limit_X_upper, angle_limit_Y_lower,  angle_limit_Y_upper);

		double z_sum_ = 0.0;
		int pointCount = 0;

		for(size_t i=0; i<latest_cloud_.points.size(); i++)
		{
			if(latest_cloud_.points[i].z > z_limit_ || latest_cloud_.points[i].z <= 0.0) continue;
			else
			{
				double x_angle = atan2(latest_cloud_.points[i].z, latest_cloud_.points[i].x);
				double y_angle = atan2(latest_cloud_.points[i].z, latest_cloud_.points[i].y);

				if(x_angle<angle_limit_X_upper && x_angle>angle_limit_X_lower && y_angle<angle_limit_Y_upper && y_angle>angle_limit_Y_lower)
				{
					z_sum_ = z_sum_ + latest_cloud_.points[i].z;
					pointCount ++;
				}
			}
		}

		if(pointCount>0) res.roi_dist = z_sum_/(double)pointCount;

		return true;
	}

	void pcl_process::plane_fitting(const PointCloudRGB::ConstPtr& pcl_in)
	{
		PointCloudRGB cloud_input = *pcl_in;
		ROS_INFO("total points: %d", cloud_input.points.size());

		//perform downsampling for the input pointcloud;
		pcl::VoxelGrid<pcl::PointXYZRGB> sor;
		PointCloudRGB::Ptr input_msg_filtered (new PointCloudRGB ());
		float downsample_size_ = 0.05;
		sor.setInputCloud(cloud_input.makeShared());
		sor.setLeafSize (downsample_size_, downsample_size_, downsample_size_);
		sor.filter (*input_msg_filtered);
		cloud_input = * input_msg_filtered;

		ROS_INFO("remained points: %d", cloud_input.points.size());

		//at most 3 planes in the pool;
		pcl::ModelCoefficients::Ptr coefficients1 (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers1 (new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients2 (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients3 (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers3 (new pcl::PointIndices);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr surface_plane1 (new pcl::PointCloud<pcl::PointXYZRGB> ());
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr surface_plane2 (new pcl::PointCloud<pcl::PointXYZRGB> ());
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr surface_plane3 (new pcl::PointCloud<pcl::PointXYZRGB> ());

		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZRGB> seg;
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (0.1);
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;

		seg.setInputCloud (cloud_input.makeShared ());
		seg.segment (*inliers1, *coefficients1);

		extract.setInputCloud (cloud_input.makeShared());
		extract.setIndices (inliers1);
		extract.setNegative (false);
		extract.filter (*surface_plane1);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr remained_points (new pcl::PointCloud<pcl::PointXYZRGB> ());
		extract.setInputCloud (cloud_input.makeShared());
		extract.setIndices (inliers1);
		extract.setNegative (true);
		extract.filter (*remained_points);

		double remained_ratio = ((double)remained_points->points.size())/((double)cloud_input.points.size());

		if(remained_ratio > 0.2)
		{
			seg.setInputCloud (remained_points->makeShared ());
			seg.segment (*inliers2, *coefficients2);
			extract.setInputCloud (remained_points->makeShared ());
			extract.setIndices (inliers2);
			extract.setNegative (false);
			extract.filter (*surface_plane2);

			extract.setInputCloud (remained_points->makeShared ());
			extract.setIndices (inliers2);
			extract.setNegative (true);
			extract.filter (*remained_points);

			remained_ratio = ((double)remained_points->points.size())/((double)cloud_input.points.size());

			if(remained_ratio > 0.2)
			{
				seg.setInputCloud (remained_points->makeShared ());
				seg.segment (*inliers3, *coefficients3);
				extract.setInputCloud (remained_points->makeShared ());
				extract.setIndices (inliers3);
				extract.setNegative (false);
				extract.filter (*surface_plane3);

				extract.setInputCloud (remained_points->makeShared ());
				extract.setIndices (inliers3);
				extract.setNegative (true);
				extract.filter (*remained_points);
			}
		}

		int plane1_type, plane2_type, plane3_type;
		plane1_type = plane_classication(*inliers1, *coefficients1, cloud_input);
		plane2_type = plane_classication(*inliers2, *coefficients2, cloud_input);
		plane3_type = plane_classication(*inliers3, *coefficients3, cloud_input);

		if(plane1_type ==1) pcl_vis_pub_.publish(*surface_plane1);
		else if(plane2_type ==1) pcl_vis_pub_.publish(*surface_plane2);
		else if(plane3_type ==1) pcl_vis_pub_.publish(*surface_plane3);
	}

	int pcl_process::plane_classication(pcl::PointIndices &plane_indices, pcl::ModelCoefficients &plane_coefs, PointCloudRGB &pcl_all)
	{
		double remained_ratio = ((double)plane_indices.indices.size())/((double)pcl_all.points.size());
		if(remained_ratio < 0.1 ||plane_indices.indices.size() <100 )return 0;

		//pay attention to the coordinate of stereo_camera;
		double normalized_z = abs(plane_coefs.values[1]);
		if(normalized_z>= 0.8) return 1;
		else return 2;
	}

	void pcl_process::check_process()
	{

	}
};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "pcl_process_node");

	 ros::NodeHandle n;
	 bbauv_pcl::pcl_process pcl_process_node;
     ros::spin();
     return 0;
}
