#include "Movement.h"
#include "Config.h"

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
		//Serial.print("Target: ");
		//Serial.println(targetOrientation);

		while (counter <= 2) {

			//Serial.print("Facing: ");
			//Serial.println(Compass.GetHeadingDegrees());

			error = (targetOrientation - Compass.GetHeadingDegrees());
			if (error < 0) errorComplement = error + 360;
			else errorComplement = error - 360;
			if (abs(errorComplement) < abs(error)) error = errorComplement;

			//Serial.print("Error: ");
			//Serial.println(error);

			// error = smallest change in degrees to target

			if (-20 < error && error <= 0) mSpeed = error * COMPASS_TURN_P_CONSTANT - (255 - 20 * COMPASS_TURN_P_CONSTANT);
			else if (0 <= error && error < 20) mSpeed = error * COMPASS_TURN_P_CONSTANT + (255 - 20 * COMPASS_TURN_P_CONSTANT);
			else if (error < 0) mSpeed = -255;
			else mSpeed = 255;

			//Serial.print("Movement Speed: ");
			//Serial.println(mSpeed);


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
