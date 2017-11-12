#include "Detection.h"
#include "Config.h"

int Detection::distanceConfig(int dist) {
  return(dist==0?200:dist-=SENSOR_OFFSET); 
  }

//Detection constructor sets up ultrasonic sensors
Detection::Detection(int trigForward, int trigAngled, int trigLeft, int echoForward, int echoAngled, int echoLeft, int maxDistance) : forwardSonar(trigForward, echoForward, maxDistance), angledSonar(trigAngled, echoAngled, maxDistance), leftSonar(trigLeft, echoLeft, maxDistance)
{
  resetDistArray();   

  maxDist = maxDistance;

}

void Detection::resetDistArray() {
  
  FwdPastDist[0] = distanceConfig(forwardSonar.ping_cm());
  FwdPastDist[1] = distanceConfig(forwardSonar.ping_cm());
  FwdPastDist[2] = distanceConfig(forwardSonar.ping_cm());
  FwdPastDist[3] = distanceConfig(forwardSonar.ping_cm());
  AngledPastDist[0] = distanceConfig(angledSonar.ping_cm());
  AngledPastDist[1] = distanceConfig(angledSonar.ping_cm());
  AngledPastDist[2] = distanceConfig(angledSonar.ping_cm());
  AngledPastDist[3] = distanceConfig(angledSonar.ping_cm());
  LeftPastDist[0] = distanceConfig(leftSonar.ping_cm());
  LeftPastDist[1] = distanceConfig(leftSonar.ping_cm());
  LeftPastDist[2] = distanceConfig(leftSonar.ping_cm());
  LeftPastDist[3] = distanceConfig(leftSonar.ping_cm());
  
  
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

   Serial.print("Past array: ");
 for (int i = 0; i < 4; i++) {
  Serial.print(pastVals[i]);
  Serial.print(", ");
  
  }
  Serial.println();
	return tempArray[2];
}

//get the distance from a sensor
int Detection::getDistance(sensorID ID) {
	if (ID == sensorID::Front) {
		int newDist = distanceConfig(forwardSonar.ping_cm());
		int dist = getMedian(FwdPastDist, newDist);

    // update array with new val, delete oldest val
    FwdPastDist[3] = FwdPastDist[2];
    FwdPastDist[2] = FwdPastDist[1];
    FwdPastDist[1] = FwdPastDist[0];
    FwdPastDist[0] = newDist;
    

		return dist;
	}
	if (ID == sensorID::Left) {
		int newDist = distanceConfig(leftSonar.ping_cm());
		int dist = getMedian(LeftPastDist, newDist);


   // update array with new val, delete oldest val
    LeftPastDist[3] = LeftPastDist[2];
    LeftPastDist[2] = LeftPastDist[1];
    LeftPastDist[1] = LeftPastDist[0];
    LeftPastDist[0] = newDist;
    
		return dist;
	}
	if (ID == sensorID::Angled) {
		int newDist = distanceConfig(angledSonar.ping_cm());
		int dist = getMedian(AngledPastDist, newDist);


   // update array with new val, delete oldest val
    AngledPastDist[3] = AngledPastDist[2];
    AngledPastDist[2] = AngledPastDist[1];
    AngledPastDist[1] = AngledPastDist[0];
    AngledPastDist[0] = newDist;
    
		return dist;
	}
}
