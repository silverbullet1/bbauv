#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;
//Function to calculate angle
double angle( Point pt1, Point pt2, Point pt0 ) {
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}
Point computeCenter(vector<vector<Point>> squares){
	double x=0.0;
	double y=0.0;
	for (int i=0;i<squares.size();i++){
		vector<Point> tmp=squares[i];
		double x_avg=0;
		double y_avg=0;
		//cout<<"Rectangle no "<<i<<endl;
		for (int j=0;j<tmp.size();j++){
			//cout<<"Point "<<j<<" x="<<tmp[j].x<<" y="<<tmp[j].y<<endl;
			x_avg+=tmp[j].x;
			y_avg+=tmp[j].y;
		}
		x_avg=x_avg/tmp.size();
		y_avg=y_avg/tmp.size();
		x+=x_avg;
		y+=y_avg;
	}
	x=x/squares.size();
	y=y/squares.size();
	Point result;
	result.x=x;
	result.y=y;
	//cout<<"Center x:="<<x<<" y:="<<y<<endl;
	return result;

}
vector<vector<Point>>findSquaresInImage(Mat _image)
{
	std::vector<std::vector<Point> > squares;
	Mat pyr, timg, gray0(_image.size(), CV_8U), gray;
	int thresh = 50, N = 11;
	//Resize then upsize to reduce noise
	pyrDown(_image, pyr, Size(_image.cols/2, _image.rows/2));
	pyrUp(pyr, timg, _image.size());
	vector<vector<Point> > contours;
	// find squares in every color plane of the image
	for( int c = 0; c < 1; c++ ) {
		// extract the c-th color plane
		int ch[] = {c, 0};
		mixChannels(&timg, 1, &gray0, 1, ch, 1);
		// try several threshold levels
		for( int l = 0; l < N; l++ ) {
			// hack: use Canny instead of zero threshold level.
			// Canny helps to catch squares with gradient shading
			if( l == 0 ) {

				// apply Canny. Take the upper threshold from slider
				// and set the lower to 0 (which forces edges merging)
				Canny(gray0, gray, 0, thresh, 5);
				dilate(gray, gray, Mat(), Point(-1,-1));
			}
			else {
				// apply threshold if l!=0:
				//     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
				gray = gray0 >= (l+1)*255/N;
			}
			// find contours and store them all as a list
			vector<Vec4i> hierarchy;


			/// Find contours
			findContours(gray, contours,hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE,Point(0, 0));

			vector<Point> approx;
			for( int i = 0; i < contours.size(); i++ )
			{
				// approximate contour with accuracy proportional
				// to the contour perimeter
				approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
				// square contours should have 4 vertices after approximation
				// relatively large area (to filter out noisy contours)
				// and be convex.
				// Note: absolute value of an area is used because
				// area may be positive or negative - in accordance with the
				// contour orientation
				double area=contourArea(Mat(approx));
				bool isConvex=isContourConvex(Mat(approx));
				if (approx.size()>=4 && approx.size()<8 && fabs(area)>100 && isConvex){
				//if( approx.size() == 4 && fabs(contourArea(Mat(approx))) > 1000 && isContourConvex(Mat(approx))) {
					double maxCosine = 0;

					// find minimum angle between joint
					// edges (maximum of cosine)
					for( int j = 2; j < 5; j++ )
					{
						double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
						maxCosine = MAX(maxCosine, cosine);
					}
					// if cosines of all angles are small
					// (all angles are ~90 degree) then write quandrange
					// vertices to resultant sequence
					if( maxCosine < 0.3 ) {
						squares.push_back(approx);
					}
					//squares.push_back(approx);
				
				}
			}
		}
	}
	return squares;
}

// the function draws all the squares in the image
void drawSquares( Mat img, vector<vector<Point>> squares)
{
	Mat img2=img;
	vector<Point> sqr;
	Point cen=computeCenter(squares);
	for (int i=0;i<squares.size();i++){
		sqr=squares[i];
		for (int j=0;j<4;j++){
			line(img2,sqr[j],sqr[(j+1)%4],Scalar(255,0,255));
		}
	}
	/*for ( int i = 0; i< squares.size(); i++ ) {
		// draw contour
		drawContours(img2, squares, i, cv::Scalar(255,0,0), 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
		// draw bounding rect
		cv::Rect rect = boundingRect(cv::Mat(squares[i]));
		cv::rectangle(img2, rect.tl(), rect.br(), cv::Scalar(0,255,0), 2, 8, 0);

		// draw rotated rect
		cv::RotatedRect minRect = minAreaRect(cv::Mat(squares[i]));
		cv::Point2f rect_points[4];
		minRect.points( rect_points );
		for ( int j = 0; j < 4; j++ ) {
			cv::line( img2, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0,0,255), 1, 8 ); // blue
		}
	}*/
	circle(img2,cen,3, Scalar(0,255,0), -1, 8, 0 );
	//namedWindow( "Rect", CV_WINDOW_AUTOSIZE );
	//imshow("Rect",img2);
}

void ComputeHough(Mat img){
	Mat edges;
	Canny(img,edges,50,200,3);
	namedWindow("Canny");
	imshow("Canny",edges);
	Mat gray;
	cvtColor(edges, gray, CV_GRAY2BGR);
	vector<Vec2f> lines;
	HoughLines(edges, lines, 1, CV_PI/180, 40, 0, 0 );
	namedWindow("detected lines");
	for( size_t i = 0; i < lines.size(); i++ )
	{
		float rho = lines[i][0], theta = lines[i][1];
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		pt1.x = cvRound(x0 + 1000*(-b));
		pt1.y = cvRound(y0 + 1000*(a));
		pt2.x = cvRound(x0 - 1000*(-b));
		pt2.y = cvRound(y0 - 1000*(a));
		line( gray, pt1, pt2, Scalar(0,0,255), 1, CV_AA);
	}
	imshow("detected lines",gray);
	Rect a;
}
int main(int argc, char* argv[]){
	int frameskip = 1;
	VideoCapture cap("E:/frames/Fuve_run.flv"); // open the default camera
	//VideoCapture cap(0); // open the default camera
	if(!cap.isOpened())  // check if we succeeded
		return -1;
	vector<vector<Point>> squares;
	Mat edges;
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
	//namedWindow("edges",1);
	for(;;)
	{
		Mat frame;
		int frames = frameskip;
		while (frames-- > 0) {
			cap >> frame;
		}
		//cvtColor(frame, edges, CV_BGR2GRAY);
		pyrDown(frame, frame, Size(frame.cols/2, frame.rows/2));
		Mat mask;
		cvtColor(frame,mask,CV_BGR2HSV);
		namedWindow("hsv",1);
		imshow("hsv",mask);
		
		Mat mask2;

		inRange(mask,Scalar(hs,ss,vs,0),Scalar(he,se,ve,0),mask2);
		erode(mask2,mask2,Mat());
		dilate(mask2,mask2,Mat());
		Mat sharp;
		//GaussianBlur( mask2, mask2, Size(9, 9), 2, 2 );
		GaussianBlur(mask2, sharp, cv::Size(9, 9), 3);
		addWeighted(mask2, 1.5, sharp, -0.5, 0, sharp);
		namedWindow("mask",1);
		imshow("mask",sharp);
		//imshow("mask",mask2);
		squares=findSquaresInImage(sharp);
		drawSquares(frame,squares);
		//ComputeHough(mask2);
		
		namedWindow("buoyCir",1);
		imshow("buoyCir",frame);
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
	}
	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}