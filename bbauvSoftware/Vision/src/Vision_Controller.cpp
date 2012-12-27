#include "Vision_Controller.h"

Vision_controll::Vision_controll(int i,string name){
	if (i==-1)
		taker=new Img_Taker(name);
	else
		taker=new Img_Taker(i);
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
	Mat orange;
	Mat green;
	Mat cur;
	Mat thres_hold_img;
	int frameskip = 1;
	reference="sword.png";
	while (1){
		int frames = frameskip;
		while (frames-- > 0) {
			frame=taker->getFrame();
		}
		
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
			frameskip += 10;
			break;
		case '-':
			frameskip -= 10;
			frameskip = std::max(0, frameskip);
			break;
		case '0':
			frameskip = 1;
			break;
		case ' ':
			frameskip = !frameskip;
			break;
		}

		if (frameskip<0)
			frameskip=0;
	}
}
