#include <ros/ros.h>
#include <image_transport/image_transport.h>
namespace golfcar_vision{
    
    class addCamInfo {

    public:
        addCamInfo(ros::NodeHandle&);
        ~addCamInfo();
        void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr);

    private:
        ros::NodeHandle n_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        ros::Publisher CamInfo_pub_;
        sensor_msgs::CameraInfo CameraStaticInfo_;
    };

    addCamInfo::addCamInfo(ros::NodeHandle &n):
    n_(n), it_(n_)
    {
        ros::NodeHandle nh;
        CamInfo_pub_ = nh.advertise<sensor_msgs::CameraInfo>("/stereo_camera/left/camera_info", 2);
        image_sub_ = it_.subscribe("/stereo_camera/left/image_raw", 1, &addCamInfo::imageCallback, this);
        
        CameraStaticInfo_.width = 640;
        CameraStaticInfo_.height = 480;
        //CameraStaticInfo_.distortion_model = "plumb_bob";
        CameraStaticInfo_.distortion_model = "rational_polynomial";
        
        CameraStaticInfo_.D.push_back(-0.218659);
        CameraStaticInfo_.D.push_back(-0.562506);
        CameraStaticInfo_.D.push_back(0.004673);
        CameraStaticInfo_.D.push_back(0.002066);
        CameraStaticInfo_.D.push_back(-1.070931);
        CameraStaticInfo_.D.push_back(0.121905);
        CameraStaticInfo_.D.push_back(-0.772331);
        CameraStaticInfo_.D.push_back(-1.304171);
        
        CameraStaticInfo_.K[0]= 453.383317;
        CameraStaticInfo_.K[1]= 0.0;
        CameraStaticInfo_.K[2]= 322.326522;
        CameraStaticInfo_.K[3]= 0.0;
        CameraStaticInfo_.K[4]= 453.379651;
        CameraStaticInfo_.K[5]= 254.378049;
        CameraStaticInfo_.K[6]= 0.0;
        CameraStaticInfo_.K[7]= 0.0;
        CameraStaticInfo_.K[8]= 1.00;
        
        CameraStaticInfo_.R[0]= 0.999707;
        CameraStaticInfo_.R[1]= 0.007199;
        CameraStaticInfo_.R[2]= -0.023121;
        CameraStaticInfo_.R[3]= -0.007310;
        CameraStaticInfo_.R[4]= 0.999962;
        CameraStaticInfo_.R[5]= -0.004697;
        CameraStaticInfo_.R[6]= 0.023087;
        CameraStaticInfo_.R[7]= 0.004864;
        CameraStaticInfo_.R[8]= 0.999722;
        
        CameraStaticInfo_.P[0]= 575.662229;
        CameraStaticInfo_.P[1]= 0.0;
        CameraStaticInfo_.P[2]= 372.393010;
        CameraStaticInfo_.P[3]= 0.0;
        CameraStaticInfo_.P[4]= 0.0;
        CameraStaticInfo_.P[5]= 575.662229;
        CameraStaticInfo_.P[6]= 252.123187;
        CameraStaticInfo_.P[7]= 0.0;
        CameraStaticInfo_.P[8]= 0.0;
        CameraStaticInfo_.P[9]= 0.0;
        CameraStaticInfo_.P[10]= 1.0;
        CameraStaticInfo_.P[11]= 0.0;
    }
    
    addCamInfo::~addCamInfo(){}
    
    void addCamInfo::imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
    {
        CameraStaticInfo_.header = msg_ptr->header;
        CameraStaticInfo_.header.frame_id = "stereo_camera";
        CamInfo_pub_.publish(CameraStaticInfo_);
        return;
    }

};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "addCamInfo_left");
	 ros::NodeHandle n;
	 golfcar_vision::addCamInfo addCamInfo_node(n);
     ros::spin();
     return 0;
}

