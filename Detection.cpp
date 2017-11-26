#include "Detection.h"
#include "Config.h"

int Detection::distanceConfig(int dist) {
	return(dist == 0 ? 200 : dist -= SENSOR_OFFSET);
}

//Detection constructor sets up ultrasonic sensors
Detection::Detection(int trigForward, int trigAngled, int trigLeft, int echoForward, int echoAngled, int echoLeft, int maxDistance) : forwardSonar(trigForward, echoForward, maxDistance), angledSonar(trigAngled, echoAngled, maxDistance), leftSonar(trigLeft, echoLeft, maxDistance)
{
	resetDistArray();

	maxDist = maxDistance;

}

int Detection::ping_mm(sensorID theSensor) {

	int uS; //microseconds

	switch (theSensor) {

	case sensorID::Front:
		uS = forwardSonar.ping();
		break;
	case sensorID::Angled:
		uS = angledSonar.ping();
		break;
	case sensorID::Left:
		uS = leftSonar.ping();
		break;
	}

	return int(uS / MICROSECONDS_PER_MM);





}

void Detection::resetDistArray() {

	FwdPastDist[0] = distanceConfig(ping_mm(sensorID::Front));
	FwdPastDist[1] = distanceConfig(ping_mm(sensorID::Front));
	FwdPastDist[2] = distanceConfig(ping_mm(sensorID::Front));
	FwdPastDist[3] = distanceConfig(ping_mm(sensorID::Front));
	AngledPastDist[0] = distanceConfig(ping_mm(sensorID::Angled));
	AngledPastDist[1] = distanceConfig(ping_mm(sensorID::Angled));
	AngledPastDist[2] = distanceConfig(ping_mm(sensorID::Angled));
	AngledPastDist[3] = distanceConfig(ping_mm(sensorID::Angled));
	LeftPastDist[0] = distanceConfig(ping_mm(sensorID::Left));
	LeftPastDist[1] = distanceConfig(ping_mm(sensorID::Left));
	LeftPastDist[2] = distanceConfig(ping_mm(sensorID::Left));
	LeftPastDist[3] = distanceConfig(ping_mm(sensorID::Left));


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

	/*   Serial.print("Past array: ");
	 for (int i = 0; i < 4; i++) {
	  Serial.print(pastVals[i]);
	  Serial.print(", ");

	  }
	  Serial.println(); */
	return tempArray[2];
}

//get the distance from a sensor
int Detection::getDistance(sensorID ID) {
	if (ID == sensorID::Front) {
		int newDist = distanceConfig(ping_mm(sensorID::Front));
		int dist = getMedian(FwdPastDist, newDist);

		// update array with new val, delete oldest val
		FwdPastDist[3] = FwdPastDist[2];
		FwdPastDist[2] = FwdPastDist[1];
		FwdPastDist[1] = FwdPastDist[0];
		FwdPastDist[0] = newDist;


		return dist;
	}
	if (ID == sensorID::Left) {
		int newDist = distanceConfig(ping_mm(sensorID::Left));
		int dist = getMedian(LeftPastDist, newDist);


		// update array with new val, delete oldest val
		LeftPastDist[3] = LeftPastDist[2];
		LeftPastDist[2] = LeftPastDist[1];
		LeftPastDist[1] = LeftPastDist[0];
		LeftPastDist[0] = newDist;

		return dist;
	}
	if (ID == sensorID::Angled) {
		int newDist = distanceConfig(ping_mm(sensorID::Angled));
		int dist = getMedian(AngledPastDist, newDist);


		// update array with new val, delete oldest val
		AngledPastDist[3] = AngledPastDist[2];
		AngledPastDist[2] = AngledPastDist[1];
		AngledPastDist[1] = AngledPastDist[0];
		AngledPastDist[0] = newDist;

		return dist;
	}
}
