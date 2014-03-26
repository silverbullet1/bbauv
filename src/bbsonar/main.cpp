#include "BBSonar.h"
#include "Utility.h"

int main(int argc, char* argv[]) {
//	cout << CV_MAJOR_VERSION << endl;
//	cout << CV_MINOR_VERSION << endl;

	Utility util;
	util.initSonar();
	util.setHeadParams();
	util.writeIntensities();
	util.processImage();
//	util.drawHistogram();
	return 0;
}
