#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;

int biggest=-1;
int CompareCont(vector<vector<Point>>contours1,vector<vector<Point>>contoursRef){
	int result=-1;
	double min=10000;
	for (int i=0;i<contours1.size();i++){
		
		double t;
			if (contourArea(contours1[i])>2000){
				t=matchShapes(contours1[0],contoursRef[biggest],1,0);
				if (t<min){
				min=t;
				result=i;
				}
			}
	}
	return result;
}
int findBiggestAread(vector<vector<Point>>contours){
	int area=-1;
	int index=-1;
	for (int i=0;i<contours.size();i++){
		vector<Point> current=contours[i];
		if (area<contourArea(current)){
			index=i;
			area=contourArea(current);
		}
	}
	return index;
}
void findCont(Mat &img1,vector<vector<Point> > &contours,vector<Vec4i> &hierarchy,Mat &output){
	//GaussianBlur(img1,output,Size(9,9),2,2);
	//dilate(output,output,Mat());
	Canny(img1,output,100,200);
	//Canny(output,output,100,200);
	/*namedWindow("canny");
	imshow("canny",img1);*/
	findContours(output,contours,hierarchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	output=output&0;
	for (int i=0;i<contours.size();i++)
		drawContours( output, contours, (int)i, Scalar(255,255,255), 1, 8, vector<Vec4i>(), 0, Point() );
}
int main(){
	VideoCapture cap(1);
	Mat frame;
	Mat mask;
	Mat refer=imread("E:/frames/sword4.png");
	namedWindow("R");
	imshow("R",refer);
	
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<RotatedRect> minRect;
	namedWindow("frame");
	namedWindow("mask");
	namedWindow("HSV Controls", CV_WINDOW_AUTOSIZE);
	resizeWindow("HSV Controls", 500, 500);

	/// Create Trackbar to choose type of Threshold

	int hs=0, he=0, ss=10, se=255, vs=10, ve=255;

	createTrackbar( "hStart: ", "HSV Controls", &hs, 180, 0);
	createTrackbar( "hEnd:   ", "HSV Controls", &he, 180, 0);
	createTrackbar( "sStart: ", "HSV Controls", &ss, 255, 0);
	createTrackbar( "sEnd:   ", "HSV Controls", &se, 255, 0);
	createTrackbar( "vStart: ", "HSV Controls", &vs, 255, 0);
	createTrackbar( "vEnd:   ", "HSV Controls", &ve, 255, 0);

	vector<vector<Point> > contoursR;
	vector<Vec4i> hierarchyR;
	findCont(refer,contoursR,hierarchyR,refer);
	biggest=findBiggestAread(contoursR);
	Mat abc;
	refer.copyTo(abc);
	abc=abc&0;
	drawContours(abc,contoursR,biggest,Scalar(255,255,255));
	namedWindow("ReferenceB");
	imshow("ReferenceB",abc);
	namedWindow("Reference");
	imshow("Reference",refer);
	Mat mask2;
	Mat frame2;
	while (1){
		minRect.clear();
		cap >> frame;
		cvtColor(frame,frame2,CV_BGR2HSV);
		//GaussianBlur(frame,frame,Size(9,9),2,2);
		inRange(frame2,Scalar(hs,ss,vs,0),Scalar(180,se,ve,0),mask);
		inRange(frame2,Scalar(0,ss,vs,0),Scalar(he,se,ve,0),mask2);
		mask=mask|mask2;
		erode(mask,mask,Mat());
		dilate(mask,mask,Mat());
		GaussianBlur(mask,mask,Size(9,9),2,2);
		Mat canny;
		/*Canny(mask,canny,100,200,3);
		namedWindow("Canny");
		imshow("Canny",canny);
		
		imshow("mask",mask);
		findContours(canny,contours,hierarchy,CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );*/
		findCont(mask,contours,hierarchy,canny);
		namedWindow("Canny");
		imshow("Canny",canny);

		imshow("mask",mask);

		int index=CompareCont(contours,contoursR);
		if (index!=-1){
			cout<<"match index "<<index<<endl;
		//for (int i=0;i<contours.size();i++)
		//{ 
			//RotatedRect tmp=minAreaRect( Mat(contours[index]) );
				//minRect.push_back(tmp);
		
		//}
		//for( size_t i = 0; i< contours.size(); i++ )
		//{
			Scalar color = Scalar( 0, 255,0 );
			// contour
			
				drawContours( frame, contours, (int)index, color, 4, 8, vector<Vec4i>(), 0, Point() );
			
			// rotated rectangle
			/*Point2f rect_points[4]; minRect[index].points( rect_points );
			for( int j = 0; j < 4; j++ )
				line( frame, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );*/
		//}
		}
		imshow("frame",frame);
		
		if(waitKey(30) >= 0) break;
	}
}