#include "Vision_Controller.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv){
	ros::init(argc, argv, "Vision_Controller");
	Vision_controll *controller=new Vision_controll();
	controller->Init();
	controller->start();
	return 0;
}
