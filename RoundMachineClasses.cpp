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
  #endif
  /*
  Compass.SetDeclination(12, 24, 'E');
  Compass.SetSamplingMode(COMPASS_SINGLE);
  Compass.SetScale(COMPASS_SCALE_130);  
  Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);
  */
}

//turn function. onSpot dictates if the robot pivots "on the spot" or around a point on the same side as it is turning towards. _degrees is the angle to turn, - = left, + = right
void Movement::turn(bool onSpot, int _degrees) {

	if (_degrees != 0) {
		#ifdef COMPASS
		int counter = 0;
		int error;
		int errorComplement;
		int mSpeed;

		int targetOrientation = (Compass.GetHeadingDegrees() + _degrees);
		if (targetOrientation < 0) targetOrientation += 360;
		else targetOrientation %= 360;

		while (counter <= 5) {
			error = (targetOrientation - Compass.GetHeadingDegrees());
			if (error < 0) errorComplement = error + 360;
			else errorComplement = error - 360;
			if (abs(errorComplement) < abs(error)) error = errorComplement;

			if (-20 < error <= 0) mSpeed = error * COMPASS_TURN_P_CONSTANT - (255 - 20 * COMPASS_TURN_P_CONSTANT);
			else if (0 <= error < 20) mSpeed = error * COMPASS_TURN_P_CONSTANT + (255 - 20 * COMPASS_TURN_P_CONSTANT);
			else if (error < 0) mSpeed = -255;
			else mSpeed = 255;

			if (-2 < error < 2) counter++;
			
			if (onSpot) drive(mSpeed, -mSpeed);
			else {	//turn on pivot
				if (error < 0) {	//turning left
					drive(abs(error) * wheelDifRatio, abs(error));
				}
				else drive(abs(error), abs(error) * wheelDifRatio);	//turning right
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
  
}

//get the distance from a sensor
int Detection::getDistance(sensorID ID) {
	if (ID == sensorID::Front){
		int travelTime = forwardSonar.ping_median(1);   //smooth the results by taking a median
    int dist = forwardSonar.convert_cm(travelTime) - FORWARD_SENSOR_OFFSET;   //subtract the offset
    if (dist == 0) dist = maxDist;  //New ping returns 0 if distance is greater than maxDist. In this case, we want to retun maxDist
    return dist;
	}
	if (ID == sensorID::Left) {
    int travelTime = leftSonar.ping_median(3);  //smooth the results by taking a median
    int dist = leftSonar.convert_cm(travelTime) - LEFT_SENSOR_OFFSET; //subtract the offset
    if (dist == 0) dist = maxDist;  //New ping returns 0 if distance is greater than maxDist. In this case, we want to retun maxDist
    return dist;
	}
	if (ID == sensorID::Angled) {
    int travelTime = angledSonar.ping_median(3);  //smooth the results by taking a median
		int dist = angledSonar.convert_cm(travelTime) - ANGLED_SENSOR_OFFSET; //subtract the offset
    if (dist == 0) dist = maxDist;  //New ping returns 0 if distance is greater than maxDist. In this case, we want to retun maxDist
    return dist;
	}
}
  //check if a new command is available over BT
bool HumanInterface::checkBT() {
	if (BTSerial.available() > 0 || MODE_BUTTON == HIGH) {
		return true;
	}
	else { 
		return false;
	}
}

//translate a command and return it
Command HumanInterface::getInput() {

  // try catch this line to see if button has been pressed
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

//HumanInterface constructor. It starts the serial port for the BT
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
=======
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
  #endif
  /*
  Compass.SetDeclination(12, 24, 'E');
  Compass.SetSamplingMode(COMPASS_SINGLE);
  Compass.SetScale(COMPASS_SCALE_130);  
  Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);
  */
}

//turn function. onSpot dictates if the robot pivots "on the spot" or around a point on the same side as it is turning towards. _degrees is the angle to turn, - = left, + = right
void Movement::turn(bool onSpot, int _degrees) {

	if (_degrees != 0) {
		#ifdef COMPASS
		startOrientation = Compass.GetHeadingDegrees(); //find out the orientation before the turn starts
   Serial.print("Start Orientation: ");
   Serial.println(startOrientation);
   Serial.print("_degrees: ");
   Serial.println(_degrees);
		if (onSpot) {
			// if turn anticlockwise
			if (_degrees < 0) {
        //target orientation must be between 0 - 360
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
  
}

//get the distance from a sensor
int Detection::getDistance(sensorID ID) {
	if (ID == sensorID::Front){
		int travelTime = forwardSonar.ping_median(1);   //smooth the results by taking a median
    int dist = forwardSonar.convert_cm(travelTime) - FORWARD_SENSOR_OFFSET;   //subtract the offset
    if (dist == 0) dist = maxDist;  //New ping returns 0 if distance is greater than maxDist. In this case, we want to retun maxDist
    return dist;
	}
	if (ID == sensorID::Left) {
    int travelTime = leftSonar.ping_median(3);  //smooth the results by taking a median
    int dist = leftSonar.convert_cm(travelTime) - LEFT_SENSOR_OFFSET; //subtract the offset
    if (dist == 0) dist = maxDist;  //New ping returns 0 if distance is greater than maxDist. In this case, we want to retun maxDist
    return dist;
	}
	if (ID == sensorID::Angled) {
    int travelTime = angledSonar.ping_median(3);  //smooth the results by taking a median
		int dist = angledSonar.convert_cm(travelTime) - ANGLED_SENSOR_OFFSET; //subtract the offset
    if (dist == 0) dist = maxDist;  //New ping returns 0 if distance is greater than maxDist. In this case, we want to retun maxDist
    return dist;
	}
}
  //check if a new command is available over BT
bool HumanInterface::checkBT() {
	if (BTSerial.available() > 0 /*|| MODE_BUTTON == HIGH*/) {
		return true;
	}
	else { 
		return false;
	}
}

//translate a command and return it
Command HumanInterface::getInput() {

  // try catch this line to see if button has been pressed
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

//HumanInterface constructor. It starts the serial port for the BT
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
