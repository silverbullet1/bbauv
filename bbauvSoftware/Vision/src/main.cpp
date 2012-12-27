#include "Img_classifier.h"
#include "Img_Storage.h"
#include "Img_Taker.h"
#include "Img_Viewer.h"
#include "Img_Processor.h"
#include "Vision_Controller.h"
string Video="green_buoy.avi";
#include <vector>
using namespace std;
using namespace cv;
// the function draws all the squares in the image
void drawSquares( Mat img, std::vector<std::vector<Point> > squares)
{
	Mat img2=img;
	vector<Point> sqr;
	//Point cen=computeCenter(squares);
	for (int i=0;i<squares.size();i++){
		sqr=squares[i];
		for (int j=0;j<4;j++){
			line(img2,sqr[j],sqr[(j+1)%4],Scalar(255,0,255));
		}
	}
	
	//circle(img2,cen,3, Scalar(0,255,0), -1, 8, 0 );
	//namedWindow( "Rect", CV_WINDOW_AUTOSIZE );
	//imshow("Rect",img2);
}
int main(int argc, char** argv){
	/*Img_Taker *taker=new Img_Taker(Video);
	Img_Storage *storage=new Img_Storage();
	Img_classfier *classy=new Img_classfier(storage);
	Img_viewer *view=new Img_viewer(storage);
	Data_Storage *data=new Data_Storage();
	Img_Processor *processor=new Img_Processor(storage,data);
	vector<vector<Point> > squares;

	string refer="sword.png";
	Mat frame;
	Mat orange;
	Mat green;
	Mat cur;
	while (1){
		frame=taker->getFrame();
		frame.copyTo(cur);
		classy->Classify_Img(frame);
		processor->find_circles(storage->getGreenThresHolding());
		view->show_Current();
		view->show_Green();
		view->show_Cirle_Frame();
		//orange=storage->getOrangeThresHolding();
		green=storage->getGreenThresHolding();
		processor->findSquaresInImage(green,true);
		squares=data->getRect();
		drawSquares(storage->getCurrent(),squares);
		namedWindow("draw square");
		imshow("draw square",green);
		
		processor->MatchShape(storage->getRedThresholing(),refer);
		view->show_Red();
		drawContours(cur,data->get_contours(),data->get_match_index(),Scalar(0,255,0), 4, 8, vector<Vec4i>(), 0, Point() );
		namedWindow("cur");
		imshow("cur",cur);
		view->show_Current();
		
		if(waitKey(30) >= 0) break;
	}*/
	ros::init(argc, argv, "Vision_Controller");
	Vision_controll *controller=new Vision_controll(-1,Video);
	controller->Init();
	controller->start();
	return 0;
}
