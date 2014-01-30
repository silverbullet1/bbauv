#include "BBSonar.h"
#include "Utility.h"

int main(int argc, char* argv[]) {
//	int retVal;
//	BBSonar bbson;
////
//	retVal = bbson.initSonar();
//	cout << "sonar initialization: " << BVTError_GetString(retVal) << endl;
//	bbson.getSetHeadParams();
//
//	retVal = bbson.retrievePing();
//	cout << "ping retrieval: " << BVTError_GetString(retVal) << endl;
//
////	retVal = bbson.retrieveRangeData();
////	cout << "rangeData retrieval: " << BVTError_GetString(retVal) << endl;
//
////	retVal = bbson.saveRangeBearingData();
////	cout << "saving range-bearing data: " << BVTError_GetString(retVal) << endl;
//
//	retVal = bbson.processPing();
//	cout << "ping processing: " << BVTError_GetString(retVal) << endl;
//
//#ifdef LOG_SONAR_DATA
//	retVal = bbson.writeSonarLog();
//	cout << "sonar data logging: " << BVTError_GetString(retVal) << endl;
//#endif
//
//	cout << "disabling ping transmission: " << BVTError_GetString(bbson.disableTransmission()) << endl;

	Utility util;
	util.initSonar();
	util.setHeadParams();
	util.writeIntensities();
//	util.processImage();
	return 0;
}
