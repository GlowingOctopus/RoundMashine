#include "Detection.h"
#include "Config.h"

//Detection constructor sets up ultrasonic sensors
Detection::Detection(int trigForward, int trigAngled, int trigLeft, int echoForward, int echoAngled, int echoLeft, int maxDist) : forwardSonar(trigForward, echoForward, maxDist), angledSonar(trigAngled, echoAngled, maxDist), leftSonar(trigLeft, echoLeft, maxDist)
{
	FwdPastDist[0] = forwardSonar.ping_cm();
	FwdPastDist[1] = forwardSonar.ping_cm();
	FwdPastDist[2] = forwardSonar.ping_cm();
	FwdPastDist[3] = forwardSonar.ping_cm();
	AngledPastDist[0] = forwardSonar.ping_cm();
	AngledPastDist[1] = forwardSonar.ping_cm();
	AngledPastDist[2] = forwardSonar.ping_cm();
	AngledPastDist[3] = forwardSonar.ping_cm();
	LeftPastDist[0] = forwardSonar.ping_cm();
	LeftPastDist[1] = forwardSonar.ping_cm();
	LeftPastDist[2] = forwardSonar.ping_cm();
	LeftPastDist[3] = forwardSonar.ping_cm();
}


int Detection::getMedian(int pastVals[], int currentDist) {

	int tempArray[] = { pastVals[0], pastVals[1], pastVals[2], pastVals[3], currentDist };

	for (int i = 0; i < 4; i++) {
		for (int o = 0; o < (5 - (i + 1)); o++) {
			if (tempArray[o] > tempArray[o + 1]) {
				int t = tempArray[o];
				tempArray[o] = tempArray[o + 1];
				tempArray[o + 1] = t;
			}
		}
	}
	return tempArray[2];
}

//get the distance from a sensor
int Detection::getDistance(sensorID ID) {
	if (ID == sensorID::Front) {
		int dist = forwardSonar.ping_cm();
		dist = getMedian(FwdPastDist, dist);

		if (dist == 0) dist = maxDist;  //New ping returns 0 if distance is greater than maxDist. In this case, we want to retun maxDist
		dist -= FORWARD_SENSOR_OFFSET;  //subtract the offset
		return dist;
	}
	if (ID == sensorID::Left) {
		int dist = leftSonar.ping_cm();
		dist = getMedian(LeftPastDist, dist);
		if (dist == 0) dist = maxDist;  //New ping returns 0 if distance is greater than maxDist. In this case, we want to retun maxDist
		dist -= LEFT_SENSOR_OFFSET;  //subtract the offset
		return dist;
	}
	if (ID == sensorID::Angled) {
		int dist = angledSonar.ping_cm();
		dist = getMedian(AngledPastDist, dist);
		if (dist == 0) dist = maxDist;  //New ping returns 0 if distance is greater than maxDist. In this case, we want to retun maxDist
		dist -= ANGLED_SENSOR_OFFSET;  //subtract the offset
		return dist;
	}
}
