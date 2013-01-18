#ifndef IMG_TAKER_BASE_H
#define IMG_TAKER_BASE_H

#include <opencv/cv.h>

class Img_Taker_Base {
public:
	virtual ~Img_Taker_Base() { }

	virtual cv::Mat getFrame() = 0;

	virtual void setFrameskip(int skip) = 0;
	virtual int getFrameskip() = 0;
};

#endif//IMG_TAKER_BASE_H
