#include "RoundMachineClasses.h"

//Movement constructor (called when a movement object is created
Movement::Movement(int L1, int L2, int R1, int R2) {
	//define the motor outputs
	leftMotor1 = L1;
	leftMotor2 = L2;
	rightMotor1 = R1;
	rightMotor2 = R2;

	//set up the pins
	pinMode(leftMotor1, OUTPUT);
	pinMode(leftMotor2, OUTPUT);
	pinMode(rightMotor1, OUTPUT);
	pinMode(rightMotor2, OUTPUT);

	//make sure the motors are off
	digitalWrite(leftMotor1, 0);
	digitalWrite(leftMotor2, 0);
	digitalWrite(rightMotor1, 0);
	digitalWrite(rightMotor2, 0);

#ifdef COMPASS
	Wire.begin();   //get the arduino ready for I2C comunication with the compass
#endif //COMPASS
}

//turn function. onSpot dictates if the robot pivots "on the spot" or around a point on the same side as it is turning towards. _degrees is the angle to turn, - = left, + = right
void Movement::turn(bool onSpot, int _degrees) {

	if (_degrees != 0) {
#ifdef COMPASS
		int counter = 0;
		int error, errorComplement, mSpeed;

		int targetOrientation = (Compass.GetHeadingDegrees() + _degrees);
    
		targetOrientation < 0 ? targetOrientation += 360 : targetOrientation %= 360;
    Serial.print("Target: ");
    Serial.println(targetOrientation);

		while (counter <= 2) {

    Serial.print("Facing: ");
    Serial.println(Compass.GetHeadingDegrees());

			error = (targetOrientation - Compass.GetHeadingDegrees());
			if (error < 0) errorComplement = error + 360;
			else errorComplement = error - 360;
			if (abs(errorComplement) < abs(error)) error = errorComplement;

			Serial.print("Error: ");
			Serial.println(error);

			// error = smallest change in degrees to target

			if (-20 < error && error <= 0) mSpeed = -40/*error * COMPASS_TURN_P_CONSTANT - (255 - 20 * COMPASS_TURN_P_CONSTANT)*/;
			else if (0 <= error && error < 20) mSpeed = 40/*error * COMPASS_TURN_P_CONSTANT + (255 - 20 * COMPASS_TURN_P_CONSTANT)*/;
			else if (error < 0) mSpeed = -255;
			else mSpeed = 255;

			Serial.print("Movement Speed: ");
			Serial.println(mSpeed);
			

			if (-2 < error && error < 2) counter++;

			if (onSpot) drive(mSpeed, -mSpeed);
			else {	//turn on pivot
				if (mSpeed < 0)  drive(abs(mSpeed) * wheelDifRatio, abs(mSpeed)); //turning left
				else drive(abs(mSpeed), abs(mSpeed) * wheelDifRatio);	//turning right
				}
			}
#endif // COMPASS

#ifndef COMPASS
		if (onSpot) {

			// turn anticlockwise
			if (_degrees < 0) {
				leftWheel(-MAX_POWER);
				rightWheel(MAX_POWER);
				delay(-_degrees * turnSpeed);
			}

			// turn clockwise
			else {
				leftWheel(MAX_POWER);
				rightWheel(-MAX_POWER);
				delay(_degrees * turnSpeed);
			}
		}

		else {    //turning around point (pivot turning)
				  //turn anticlockwise
			if (_degrees < 0) {
				leftWheel(MAX_POWER * wheelDifRatio);
				rightWheel(MAX_POWER);
				delay(-_degrees * turnSpeed);
			}

			//turn clockwise
			else {
				leftWheel(MAX_POWER);
				rightWheel(MAX_POWER * wheelDifRatio);
				delay(_degrees * turnSpeed);
			}
		}
#endif //not COMPASS
	}
}

//stop the robot
void Movement::stop_movement() {
	leftWheel(0);
	rightWheel(0);
}

//drive the robot forwards
void Movement::drive(int _speed) {
	leftWheel(_speed);
	rightWheel(_speed);

}

//drive the robot forwards with individual control over both motors
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

  int tempArray[] = {pastVals[0], pastVals[1], pastVals[2], pastVals[3], currentDist};

    for(int i=0; i<4; i++) {
        for(int o=0; o<(5-(i+1)); o++) {
            if(tempArray[o] > tempArray[o+1]) {
               int t = tempArray[o];
               tempArray[o] = tempArray[o+1];
               tempArray[o+1] = t;
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
//check if a new command is available over BT
bool HumanInterface::checkBT() {
	if (Serial.available() > 0) { //MODE_BUTTON
		return true;
	}
	else {
		return false;
	}
}

//translate a command and return it
Command HumanInterface::getInput() {

	// try catch this line to see if button has been pressed
	char key = Serial.read();
 Serial.print('R'); //Recieved: 
 Serial.println(key);

	Command input;
	switch (key) {
	case 0x38: // 8
		input = Command::Forwards;
    Serial.println("unput = forwards");
		break;
	case 0x32: // 2
		input = Command::Backwards;
		break;
	case 0x34:// 4
		input = Command::Left;
		break;
	case 0x36: // 6
		input = Command::Right;
		break;
	case 0x44: // D
		input = Command::Release;
		break;
	case 0x43: // C
		input = Command::Grab;
		break;
	case 0x45: // E
		input = Command::RLeft;
		break;
	case 0x46: // F
		input = Command::RRight;
		break;
  default:
    input = Command::Stop;
	}
	return input;
}

//HumanInterface constructor. It starts the serial port for the BT
HumanInterface::HumanInterface(int rx, int tx) {
	
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
#endif // COMPASS

less old:
if (onSpot) {
}

startOrientation = Compass.GetHeadingDegrees(); //find out the orientation before the turn starts
if (onSpot) {
// if turn anticlockwise
if (_degrees < 0) {
//target orientation must be between 0 - 360. make it between 0 and 360 if it's not
if (startOrientation + _degrees < 0) {targetOrientation = startOrientation + _degrees + 360;}
else {targetOrientation = startOrientation + _degrees;}
Serial.print("target orientation: ");
Serial.println(targetOrientation);

while (Compass.GetHeadingDegrees() > targetOrientation || (Compass.GetHeadingDegrees() > startOrientation + _degrees && Compass.GetHeadingDegrees() < startOrientation + 3)) { //turn until the target orientation is reached

leftWheel(-MAX_POWER);
rightWheel(MAX_POWER);
Serial.print("Current orientation: ");
Serial.println(Compass.GetHeadingDegrees());
}
}

// if turn clockwise
else {
if (startOrientation + _degrees > 360) targetOrientation = startOrientation + _degrees - 360;
else targetOrientation = startOrientation + _degrees;
while (Compass.GetHeadingDegrees() < targetOrientation || (Compass.GetHeadingDegrees() < startOrientation + _degrees && Compass.GetHeadingDegrees() > startOrientation - 3)) {
leftWheel(MAX_POWER);
rightWheel(-MAX_POWER);
}
}
}

else {    //turning around point (pivot turning)
//turn anticlockwise
if (_degrees < 0) {
if (startOrientation + _degrees < 0) targetOrientation = startOrientation + _degrees + 360;
else targetOrientation = startOrientation + _degrees;
while (Compass.GetHeadingDegrees() > targetOrientation || (Compass.GetHeadingDegrees() > startOrientation + _degrees && Compass.GetHeadingDegrees() < startOrientation + 3)) {
leftWheel(MAX_POWER * wheelDifRatio);
rightWheel(MAX_POWER);
}
}

//turn clockwise
else {
if (startOrientation + _degrees > 360) targetOrientation = startOrientation + _degrees - 360;
else targetOrientation = startOrientation + _degrees;
while (Compass.GetHeadingDegrees() < targetOrientation || (Compass.GetHeadingDegrees() < startOrientation + _degrees && Compass.GetHeadingDegrees() > startOrientation - 3)) {
leftWheel(MAX_POWER);
rightWheel(MAX_POWER * wheelDifRatio);
}
}
}

*/
