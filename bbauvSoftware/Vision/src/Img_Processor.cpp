#include "Img_Processor.h"

Img_Processor::Img_Processor(Img_Storage *store,Data_Storage *data_st){
	storage=store;
	data_store=data_st;
}

Mat Img_Processor::pre_process(Mat img){
	Mat resutl;
	erode(img,resutl,Mat());
	dilate(resutl,resutl,Mat());
	GaussianBlur( resutl, resutl, Size(9, 9), 2, 2 );
	return resutl;
}
vector<vector<Point> > Img_Processor::Average_Squares(vector<vector<Point> > squares){
	vector<vector<Point> > resutl;
	resutl.clear();
	int size=squares.size();
	for (int i=0;i<size;i++)
		clockWisePoint(squares[i]);
	for (int i=0;i<size;i++){
		if (squares[i][0].x==-1) continue;
		for (int j=0;j<size;j++){
			if (squares[j][0].x==-1) continue;
			if (i!=j){
				vector<Point > P1=squares[i];
				vector<Point > P2=squares[j];
				if (Near(P1,P2)){
					for (int k=0;k<4;k++){
						squares[i][k].x=(squares[i][k].x+squares[j][k].x)/2;
						squares[j][k].y=-1;
					}
				}
			}
		}
	}
	for (int i=0;i<size;i++){
		if (squares[i][0].x==-1) continue;
		resutl.push_back(squares[i]);
	}
	return resutl;
}
void Img_Processor::clockWisePoint(vector<Point > &P1){
	vector<Point> result(4);
	int topMax=-1;
	int topId;
	for (int i=0;i<4;i++){
		if (P1[i].y > topMax){
			topMax=P1[i].y;
			topId=i;
		}
	}
	int nearMin=10000;
	int nearTopId;
	for (int i=0;i<4;i++){
		if (i!=topId){
			int dis=topMax-P1[i].y;
			if (nearMin>dis){
				nearMin=dis;
				nearTopId=i;
			}
		}
	}
	if ((P1[nearTopId].x) < (P1[topId].x)){
		result[0]=P1[nearTopId];
		result[1]=P1[topId];
	}else{
		result[1]=P1[nearTopId];
		result[0]=P1[topId];
	}

	int remain1=0;
	int remain2=1;
	for (int i=0;i<4;i++){
		if (i!=nearTopId && i!=topId){
			remain1=i;
			break;
		}
	}
	for (int i=0;i<4;i++){
		if (i!=nearTopId && i!=topId && i!=remain1){
			remain2=i;
			break;
		}
	}

	if (P1[remain1].x < P1[remain2].x){
		result[2]=P1[remain2];
		result[3]=P1[remain1];
	}else{
		result[3]=P1[remain2];
		result[2]=P1[remain1];
	}
	
	P1=result;
}
bool Img_Processor::Near(vector<Point >P1,vector<Point > P2){
	bool result=false;
	

	double d1=((P1[0].x - P2[0].x)*(P1[0].x-P2[0].x) + (P1[0].y - P2[0].y)*(P1[0].y -P2[0].y));
	double d2=((P1[1].x - P2[1].x)*(P1[1].x-P2[1].x) + (P1[1].y - P2[1].y)*(P1[1].y -P2[1].y));
	double d3=((P1[2].x - P2[2].x)*(P1[2].x-P2[2].x) + (P1[2].y - P2[2].y)*(P1[2].y -P2[2].y));
	double d4=((P1[3].x - P2[3].x)*(P1[3].x-P2[3].x) + (P1[3].y - P2[3].y)*(P1[3].y -P2[3].y));

	if (d1<=20*20 && d2<=20*20 && d3<=20*20 && d4<=20*20)
		result=true;

	return result;
}
void Img_Processor::find_circles(Mat img){
	vector<Vec3f> circles;
	Mat pre_pro=pre_process(img);
	HoughCircles(pre_pro,circles,CV_HOUGH_GRADIENT,1,pre_pro.rows/8,300,20,10,400);
	Mat cir;
	storage->getCurrent().copyTo(cir);
	for( size_t i = 0; i < circles.size(); i++ )
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		// circle center
		circle( cir, center, 3, Scalar(0,255,0), -1, 8, 0 );
		// circle outline
		circle( cir, center, radius, Scalar(0,0,255), 3, 8, 0 );
	}
	data_store->storeCircleData(circles);
	storage->storeCircleFrame(cir);
}


//Function to calculate angle (return as Cos of angle). pt0 is the intersection.
double Img_Processor::angle(Point pt1, Point pt2, Point pt0 ){
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

vector<vector<Point> > Img_Processor::findSquaresInImage(Mat _image,bool binary){
	_image=pre_process(_image);
	int channel;
	if (binary)
		channel=1;
	else
		channel=3;
	vector<vector<Point> > squares;
	Mat pyr, timg, gray0(_image.size(), CV_8U), gray;
	int thresh = 50, N = 11;
	//Resize then upsize to reduce noise
	pyrDown(_image, pyr, Size(_image.cols/2, _image.rows/2));
	pyrUp(pyr, timg, _image.size());
	vector<vector<Point> > contours;
	// find squares in every color plane of the image
	for( int c = 0; c < channel; c++ ) {
		// extract the c-th color plane
		int ch[] = {c, 0};
		mixChannels(& timg, 1, & gray0, 1, ch, 1);
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
			vector < Vec4i > hierarchy;


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
				if (approx.size()==4 && fabs(area)>100 && isConvex){
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
					if( maxCosine < 0.4 ) {
						squares.push_back(approx);
					}
					//squares.push_back(approx);

				}
			}
		}
	}
	
	vector<vector<Point> > resutl=Average_Squares(squares);
	data_store->storeRec(resutl);
	return squares;
}

//img input is binary
void Img_Processor::MatchShape(Mat img,string reference){
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Mat pre_img=pre_process(img);
	Mat refer=imread(reference);
	vector<vector<Point> > contoursR;
	vector<Vec4i> hierarchyR;
	findCont(refer,contoursR,hierarchyR,refer);
	namedWindow("Reference");
	imshow("Reference",refer);

	int biggest;
	biggest=findBiggestAread(contoursR);


	Mat canny;
	findCont(pre_img,contours,hierarchy,canny);
	int index=CompareCont(contours,contoursR,biggest);
	data_store->storeContour(contours);
	if (index!=-1){
		//cout<<"match index "<<index<<endl;
		data_store->store_match_index(index);
	}

}

// the function draws all the squares in the image
void Img_Processor::drawSquares( Mat img, vector<vector<Point> > squares)
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
}

void Img_Processor::findCont(Mat & img1,vector<vector<Point> > &contours,vector<Vec4i> & hierarchy,Mat & output){
	
	Canny(img1,output,100,200);
	
	findContours(output,contours,hierarchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	//output=output & 0;
	output.empty();
	for (int i=0;i<contours.size();i++)
		drawContours( output, contours, (int)i, Scalar(255,255,255), 1, 8, vector<Vec4i>(), 0, Point() );
}

int Img_Processor::findBiggestAread(vector<vector<Point> >contours){
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

int Img_Processor::CompareCont(vector<vector<Point> >contours1,vector<vector<Point> >contoursRef,int biggest){
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
