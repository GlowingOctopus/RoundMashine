/*
MOVEMENT CLASS (subsystem)
This class is responsible for controling the vehicle's motion
*/

#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "config.h"	//include the file with configurable parameters

class Movement {
private:
	void leftWheel(int _speed);		//function that directly controls the left wheel's motion
	void rightWheel(int _speed);	//function that directly controls the right wheel's motion

	//ints to store the pins that the hbridge is connected to
	int leftMotor1;
	int leftMotor2;
	int rightMotor1;
	int rightMotor2;

#ifdef COMPASS
	//if the COMPASS is being used, intialise it
	HMC5883L_Simple Compass;	//make an object called Compass
	float startOrientation;
	float targetOrientation;

#endif //COMPASS

public:
	//methods to control movement
	void turn(bool onSpot, int _degrees);
	void stop_movement();
	void drive(int _speed);
	void drive(int leftSpeed, int rightSpeed);

	//Constructor. L1, L2, R1 and R2 are the pins connected to the hbridge
	Movement(int L1, int L2, int R1, int R2);
};

#endif
