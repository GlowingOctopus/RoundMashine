#ifndef ROUNDMACHINE_H
#define ROUNDMACHINE_H

//#define COMPASS

#include <SoftwareSerial.h>
#include <NewPing.h>

#ifdef COMPASS
#include <Wire.h>
#include <HMC5883L_Simple.h>
//#define COMPASS_ADDRESS 0x1E //0011110b, I2C 7bit address of HMC5883
#endif // COMPASS


#define BLOCK_IN_CM 26
#define MAX_POWER 255

#define DISTANCE_TO_WALL 2        //aim to keep this gap to the wall
#define CHASSIS_DIAMETER 10
#define FORWARD_SENSOR_OFFSET 1   //distance between edge of chassis and sensor. default: 8
#define LEFT_SENSOR_OFFSET  6     //distance between edge of chassis and sensor. default: 8
#define ANGLED_SENSOR_OFFSET  0   //distance between edge of chassis and sensor. default: 8

const double wheelDifRatio =  DISTANCE_TO_WALL / (DISTANCE_TO_WALL + CHASSIS_DIAMETER);

const int experimentalNumber = 3; //the speed which the vehicle turns at in mS/degrees at full speed

enum class sensorID { Front, Left, Angled };
enum class Command { Forwards, Backwards, Left, Right, Grab, Release, RLeft, RRight };

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

  //int getOrientation();
  #endif //COMPASS

public:
  void turn(bool onSpot, int _degrees);
  void stop_movement();
  void drive(int _speed);
  void drive(int leftSpeed, int rightSpeed);

  Movement(int L1, int L2, int R1, int R2);
};

class Detection {
private:
  NewPing forwardSonar;
  NewPing angledSonar;
  NewPing leftSonar;

public:
  int getDistance(sensorID ID);
  Detection(int trigForward, int trigAngled, int trigLeft, int echoForward, int echoAngled, int echoLeft, int maxDist);
};

class HumanInterface {
private:
  SoftwareSerial BTSerial;
public:
  // Checks whether there is any new input from a user
  bool checkBT();

  // change to enum later on
  Command getInput();

  HumanInterface(int rx, int tx);
};

#endif

