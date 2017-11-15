#ifndef DETECTION_H
#define DETECTION_H

#include "config.h"

class Detection {
private:
	NewPing forwardSonar;
	NewPing angledSonar;
	NewPing leftSonar;

	int FwdPastDist[4], AngledPastDist[4], LeftPastDist[4];
	int maxDist;

	int getMedian(int pastVals[], int currentDist);
  int distanceConfig(int dist);
  int ping_mm(sensorID theSensor);

public:
	int getDistance(sensorID ID);
	Detection(int trigForward, int trigAngled, int trigLeft, int echoForward, int echoAngled, int echoLeft, int maxDist);

  void resetDistArray();


};

#endif
