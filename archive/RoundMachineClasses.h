#ifndef ROUNDMACHINE_H
#define ROUNDMACHINE_H

#define COMPASS   // define if compass is being used to manage turning

#define MODE_BUTTON A3

#include <SoftwareSerial.h>   //for BT comunication
#include <NewPing.h>          //for ultrasonic sensors

#ifdef COMPASS
#include <Wire.h>             //for I2C comunication with compass
#include <HMC5883L_Simple.h>  //for interfacing with compass
#endif // COMPASS

#define BLOCK_IN_CM 26  //size of a grid square in the maze
#define MAX_POWER 255   //default 255
#define ADJUST_POWER 200

#define COMPASS_TURN_P_CONSTANT 3	//proportional constant for turn() function

#define DISTANCE_TO_WALL 2        //aim to keep this gap to the wall
#define CHASSIS_DIAMETER 10       //diameter of robot
#define FORWARD_SENSOR_OFFSET 8   //distance between edge of chassis and sensor. default: 8
#define LEFT_SENSOR_OFFSET  8     //distance between edge of chassis and sensor. default: 8
#define ANGLED_SENSOR_OFFSET  8   //distance between edge of chassis and sensor. default: 8

const double wheelDifRatio = DISTANCE_TO_WALL / (DISTANCE_TO_WALL + CHASSIS_DIAMETER);

const int turnSpeed = 3; //the speed which the vehicle turns at in mS/degrees at full speed

enum class sensorID { Front, Left, Angled };  //e number to specify which ultrasonic sensor
enum class Command { Forwards, Backwards, Left, Right, Grab, Release, RLeft, RRight, Stop };  //e number to store commands from user
enum class State { Fwd, SlightR, SlightL, R90, L90, L45, R45, R135, UTurn, SlightFwd, DoNotChange }; //e number to define the current state of the robot

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

class Detection {
private:
	NewPing forwardSonar;
	NewPing angledSonar;
	NewPing leftSonar;

  int FwdPastDist[4], AngledPastDist[4], LeftPastDist[4];
	int maxDist;

  int getMedian(int pastVals[], int currentDist);

public:
	int getDistance(sensorID ID);
	Detection(int trigForward, int trigAngled, int trigLeft, int echoForward, int echoAngled, int echoLeft, int maxDist);


};

class HumanInterface {
private:
	//SoftwareSerial BTSerial;
public:
	// Checks whether there is any new input from a user
	bool checkBT();

	Command getInput();

	//Constructor
	HumanInterface(int rx, int tx);
};

#endif
