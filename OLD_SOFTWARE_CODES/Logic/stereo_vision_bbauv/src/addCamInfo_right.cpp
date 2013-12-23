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
        CamInfo_pub_ = nh.advertise<sensor_msgs::CameraInfo>("/stereo_camera/right/camera_info", 2);
        image_sub_ = it_.subscribe("/stereo_camera/right/image_raw", 1, &addCamInfo::imageCallback, this);
        
        CameraStaticInfo_.width = 640;
        CameraStaticInfo_.height = 480;
        //CameraStaticInfo_.distortion_model = "plumb_bob";
        CameraStaticInfo_.distortion_model = "rational_polynomial";
        
        
        
        CameraStaticInfo_.D.push_back(-0.160889);
        CameraStaticInfo_.D.push_back(0.188305);
        CameraStaticInfo_.D.push_back(0.005417);
        CameraStaticInfo_.D.push_back(0.001233);
        CameraStaticInfo_.D.push_back(-0.808047);
        CameraStaticInfo_.D.push_back(0.139877);
        CameraStaticInfo_.D.push_back(0.189066);
        CameraStaticInfo_.D.push_back(-1.028826);

        CameraStaticInfo_.K[0]= 454.163667;
        CameraStaticInfo_.K[1]= 0.0;
        CameraStaticInfo_.K[2]= 321.348288;
        CameraStaticInfo_.K[3]= 0.0;
        CameraStaticInfo_.K[4]= 454.208500;
        CameraStaticInfo_.K[5]= 254.058703;
        CameraStaticInfo_.K[6]= 0.0;
        CameraStaticInfo_.K[7]= 0.0;
        CameraStaticInfo_.K[8]= 1.00;
        
        CameraStaticInfo_.R[0]= 0.999374;
        CameraStaticInfo_.R[1]= 0.002675;
        CameraStaticInfo_.R[2]= -0.035289;
        CameraStaticInfo_.R[3]= -0.002506;
        CameraStaticInfo_.R[4]= 0.999985;
        CameraStaticInfo_.R[5]= 0.004826;
        CameraStaticInfo_.R[6]= 0.035301;
        CameraStaticInfo_.R[7]= -0.004734;
        CameraStaticInfo_.R[8]= 0.999366;
        
        CameraStaticInfo_.P[0]= 575.662229;
        CameraStaticInfo_.P[1]= 0.0;
        CameraStaticInfo_.P[2]= 372.393010;
        CameraStaticInfo_.P[3]= -68.999084;
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
	 ros::init(argc, argv, "addCamInfo_right");
	 ros::NodeHandle n;
	 golfcar_vision::addCamInfo addCamInfo_node(n);
     ros::spin();
     return 0;
}

