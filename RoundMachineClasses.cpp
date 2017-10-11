#include "RoundMachineClasses.h"

void Movement::turn(bool onSpot, int _degrees) {

	if (_degrees != 0) {
		#ifdef COMPASS
		startOrientation = getOrientation();
		if (onSpot) {
			// turn anticlockwise
			if (_degrees < 0) {
				while (getOrientation() > startOrientation + _degrees) { 
					leftWheel(-MAX_POWER);
					rightWheel(MAX_POWER);
				}
			}

			// turn clockwise
			else {
				while (getOrientation() < startOrientation + _degrees) {
					leftWheel(MAX_POWER);
					rightWheel(-MAX_POWER);
				}
			}
		}

		else {    //turning around point (pivot turning)
				  //turn anticlockwise
			if (_degrees < 0) {
				while (getOrientation() > startOrientation + _degrees) {
					leftWheel(MAX_POWER * wheelDifRatio);
					rightWheel(MAX_POWER);
				}
			}

			//turn clockwise
			else {
				while (getOrientation() < startOrientation + _degrees) {
					leftWheel(MAX_POWER);
					rightWheel(MAX_POWER * wheelDifRatio);
				}
			}
		}
		#endif // COMPASS

		#ifndef COMPASS
		if (onSpot) {

			// turn anticlockwise
			if (_degrees < 0) {
				leftWheel(-MAX_POWER);
				rightWheel(MAX_POWER);
				delay(-_degrees * experimentalNumber);
			}

			// turn clockwise
			else {
				leftWheel(MAX_POWER);
				rightWheel(-MAX_POWER);
				delay(_degrees * experimentalNumber);
			}
		}

		else {    //turning around point (pivot turning)
				  //turn anticlockwise
			if (_degrees < 0) {
				leftWheel(MAX_POWER * wheelDifRatio);
				rightWheel(MAX_POWER);
				delay(-_degrees * experimentalNumber);
			}

			//turn clockwise
			else {
				leftWheel(MAX_POWER);
				rightWheel(MAX_POWER * wheelDifRatio);
				delay(_degrees * experimentalNumber);
			}
		}
		#endif //not COMPASS
	}
}

void Movement::stop_movement() {
	leftWheel(0);
	rightWheel(0);
}

void Movement::drive(int _speed) {
	leftWheel(_speed);
	rightWheel(_speed);

}

void Movement::drive(int leftSpeed, int rightSpeed) {
	leftWheel(leftSpeed);
	rightWheel(rightSpeed);
}

void Movement::leftWheel(int _speed) {
  if (_speed >= 0) {
	analogWrite(leftMotor1, _speed);
	digitalWrite(leftMotor2, LOW);
  }
  
  else {
  digitalWrite(leftMotor1, LOW);
  analogWrite(leftMotor2, -_speed);
  }
}

void Movement::rightWheel(int _speed) {
	if (_speed >= 0) {
  analogWrite(rightMotor1, _speed);
  digitalWrite(rightMotor2, LOW);
  }
  
  else {
  digitalWrite(rightMotor1, LOW);
  analogWrite(rightMotor2, -_speed);
  }
}

Movement::Movement(int L1, int L2, int R1, int R2) {
  leftMotor1 = L1;
  leftMotor2 = L2;
  rightMotor1 = R1;
  rightMotor2 = R2;

  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);

  digitalWrite(leftMotor1, 0);
  digitalWrite(leftMotor2, 0);
  digitalWrite(rightMotor1, 0);
  digitalWrite(rightMotor2, 0);

#ifdef COMPASS
  Wire.begin();
  HMC5883L_Simple Compass;
  Compass.SetSamplingMode(COMPASS_CONTINUOUS);
#endif // COMPASS
}

#ifdef COMPASS
int Movement::getOrientation() {
	float angle= Compass.GetHeadingDegrees();
	return angle;
}
#endif

Detection::Detection(int trigForward, int trigAngled, int trigLeft, int echoForward, int echoAngled, int echoLeft, int maxDist) : forwardSonar(trigForward, echoForward, maxDist), angledSonar(trigAngled, echoAngled, maxDist), leftSonar(trigLeft, echoLeft, maxDist) 
{
  
}

int Detection::getDistance(sensorID ID) {
	if (ID == sensorID::Front){
		int travelTime = forwardSonar.ping_median(3);
    int dist = forwardSonar.convert_cm(travelTime) - FORWARD_SENSOR_OFFSET;
    if (dist == 0) dist = 200;
    return dist;
	}
	if (ID == sensorID::Left) {
    int travelTime = leftSonar.ping_median(3);
    int dist = leftSonar.convert_cm(travelTime) - LEFT_SENSOR_OFFSET;
    if (dist == 0) dist = 200;
    return dist;
	}
	if (ID == sensorID::Angled) {
    int travelTime = angledSonar.ping_median(3);
		int dist = angledSonar.convert_cm(travelTime) - ANGLED_SENSOR_OFFSET;
    if (dist == 0) dist = 200;
    return dist;
	}
}

bool HumanInterface::checkBT() {
	if (BTSerial.available() > 0) {
		return true;
	}
	else { 
		return false;
	}
}

Command HumanInterface::getInput() {
	char key = BTSerial.read();
	Command input;
	switch (key) {
		case 0x38:
			input = Command::Forwards;
			break;
		case 0x32:
			input = Command::Backwards;
			break;
		case 0x34:
			input = Command::Left;
			break;
		case 0x36:
			input = Command::Right;
			break;
		case 0x44:
			input = Command::Release;
			break;
		case 0x43:
			input = Command::Grab;
			break;
		case 0x45:
			input = Command::RLeft;
			break;
		case 0x46:
			input = Command::RRight;
			break;
	}
	return input;
}

HumanInterface::HumanInterface(int rx, int tx) : BTSerial(rx, tx) {
  BTSerial.begin(9600);
}

class Output {
private:

public:
	void setLED();

};

/* OLD
#ifdef COMPASS
  Wire.begin();
  Wire.beginTransmission(COMPASS_ADDRESS); //open communication with HMC5883
  Wire.send(0x02); //select mode register
  Wire.send(0x10); //idle mode
  Wire.endTransmission();
#endif // COMPASS
}

#ifdef COMPASS
int Movement::getOrientation() {
  int angle;
  Wire.beginTransmission(COMPASS_ADDRESS);
  Wire.send(0x03); //select register 3, X MSB register
  Wire.endTransmission();

  Wire.requestFrom(COMPASS_ADDRESS, 2);
  if (2 <= Wire.available()) {
    angle = Wire.receive() << 8; //X msb
    angle |= Wire.receive(); //X lsb
  }
  return angle;
   
}

void calibrateCompass() {} //need to measure the magnetic field for about 5 seconds to work out which way north is when the robot is powered up
#endif // COMPASS*/
