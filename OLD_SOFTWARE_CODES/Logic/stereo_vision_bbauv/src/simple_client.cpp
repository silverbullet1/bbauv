#include "ros/ros.h"
#include <cstdlib>
#include "stereo_vision_bbauv/roiDist.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_vision_client");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<stereo_vision_bbauv::roiDist>("/calcRoiDist");
  stereo_vision_bbauv::roiDist srv;
  
  //srv.request.topleft_x  = opencv_rect.origin.x;
  
  srv.request.topleft_x = 200;
  srv.request.topleft_y = 200;
  srv.request.rect_width = 100;
  srv.request.rect_height = 100;

  if (client.call(srv))
  {
    double dist = srv.response.roi_dist;
    ROS_INFO("object dist: %f", dist);
  }
  else
  {
    ROS_ERROR("service failure");
    return 1;
  }

  return 0;
}
