#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "config.h"

class Movement {
private:
	void leftWheel(int _speed);
	void rightWheel(int _speed);

	int leftMotor1;
	int leftMotor2;
	int rightMotor1;
	int rightMotor2;

#ifdef COMPASS
	HMC5883L_Simple Compass;
	float startOrientation;
	float targetOrientation;

	//int getOrientation();
#endif //COMPASS

public:
	//functions to control movement
	void turn(bool onSpot, int _degrees);
	void stop_movement();
	void drive(int _speed);
	void drive(int leftSpeed, int rightSpeed);

	//Constructor
	Movement(int L1, int L2, int R1, int R2);
};

#endif
