#include "Vision_Controller.h"
#include <dynamic_reconfigure/server.h>
#include "Vision/VisionConfig.h"
#include "Img_Taker.h"
#include "Img_Subscriber.h"

void dynamic_reconfig_callback(Vision_controll *vctl, Vision::VisionConfig &config, uint32_t level)
{
	vctl->mode_controll = config.mode_param;
	vctl->color_controll = config.color_param;
	vctl->reference = config.reference_image;

	setTrackbarPos("mode_controll", "Mode", config.mode_param);
	setTrackbarPos("color_controll", "Color", config.color_param);
}

Vision_controll::Vision_controll(){
	// Get some private params at the start
	string topic_name, video_name;
	int webcam_id;
	ros::NodeHandle private_nh("~");
	private_nh.param<string>("topic_name", topic_name, "");
	private_nh.param<string>("video_name", video_name, "");
	private_nh.param<int>("webcam_id", webcam_id, 0);

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
	storage=new Img_Storage();
	classy=new Img_classfier(storage);
	view=new Img_viewer(storage);
	data=new Data_Storage();
	processor=new Img_Processor(storage,data);
	pub = nh_.advertise<bbauv_msgs::circles>("array", 100);
}

Vision_controll::~Vision_controll(){
	delete taker;
	delete storage;
	delete classy;
	delete view;
	delete data;
	delete processor;
}

void Vision_controll::publish_Cir(){
	vector< Vec3f > cirs;
	cirr.x.clear();
	cirr.y.clear();
	cirr.radius.clear();
	cirs=data->getCircleData();
	for (int i=0;i<cirs.size();i++){
		cout<<"X Y Radius "<< cirs[i][0]<<" "<<cirs[i][1]<<" "<<cirs[i][2]<<endl;
		cirr.x.push_back(cirs[i][0]);
		cirr.y.push_back(cirs[i][1]);
		cirr.radius.push_back(cirs[i][2]);
	}
	int size=cirs.size();
	cirr.size=size;
	if (size>=1)
		pub.publish(cirr);
}

void Vision_controll::Init(){
	mode_controll=0;
	color_controll=0;
	namedWindow("Mode");
	namedWindow("Color");
	createTrackbar("mode_controll","Mode",&mode_controll,2);
	createTrackbar("color_controll","Color",&color_controll,2);
}

Mat Vision_controll::consider_Thres(){
	Mat result;
	switch(color_controll){
	case GREEN:
		result=storage->getGreenThresHolding();
		break;
	case RED:
		result=storage->getRedThresholing();
		break;
	case ORANGE:
		result=storage->getOrangeThresHolding();
		break;
	
	}
	return result;
}

void Vision_controll::Process_Mode(Mat &img,string refer,Mat &thres){
	vector<vector<Point> > squares;
	switch(mode_controll){
	case CIRCLE:
		processor->find_circles(thres);
		view->show_Cirle_Frame();
		publish_Cir();
		break;
	case RECTANGLE:
		
		processor->findSquaresInImage(thres,true);
		squares=data->getRect();
		processor->drawSquares(storage->getCurrent(),squares);
		break;
	case MATCH_SHAPE:

		processor->MatchShape(thres,refer);
		int match=data->get_match_index();
		//cout<<"Match "<<match<<endl;
		int size=data->get_contours().size();
		if (size>=0 && match<=size)
			drawContours(img,data->get_contours(),match,Scalar(0,255,0), 4, 8, vector<Vec4i>(), 0, Point() );
		storage->storeCurrent(img);
		break;
	}
}
void Vision_controll::start(){
	Mat frame;
	Mat cur;
	Mat thres_hold_img;

	reference="sword.png";

	dynamic_reconfigure::Server<Vision::VisionConfig> srv;
	dynamic_reconfigure::Server<Vision::VisionConfig>::CallbackType f;
	f = boost::bind(&dynamic_reconfig_callback, this, _1, _2);
	srv.setCallback(f);

	while (1){
		frame = taker->getFrame();
		
		frame.copyTo(cur);
		classy->Classify_Img(frame);

		thres_hold_img=consider_Thres();
		Process_Mode(frame,reference,thres_hold_img);
		view->show_Current();
		view->show_Thres(color_controll);
		/*namedWindow("Thres");
		imshow("Thres",thres_hold_img);*/
		/*namedWindow("Cur");
		imshow("Cur",cur);*/
		int c;
		c = waitKey( 20 );
		if( (char)c == 27 )
		{ break; }
		switch (c) {
		case '=':
			taker->setFrameskip(taker->getFrameskip() + 10);
			break;
		case '-':
			taker->setFrameskip(taker->getFrameskip() - 10);
			break;
		case '0':
			taker->setFrameskip(1);
			break;
		case ' ':
			taker->setFrameskip(!taker->getFrameskip());
			break;
		}

		ros::spinOnce();
	}
}
