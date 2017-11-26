#ifndef CONFIG_H
#define CONFIG_H
//Definitions of configurable variables and states

//#define DEBUGGING
#define NEW_INTERFACE

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
#define AUTO_POWER 200

#define COMPASS_TURN_P_CONSTANT 3	//proportional constant for turn() function

#define MIN_DISTANCE_TO_WALL 30        //aim to keep this gap to the wall
#define MAX_DISTANCE_TO_WALL 30        //aim to keep this gap to the wall
#define CHASSIS_DIAMETER 100       //diameter of robot
#define SENSOR_OFFSET 80   //distance between edge of chassis and sensor

#define DIST_THRESHHOLD 30

#define MICROSECONDS_PER_MM 5.7 // time in microseconds it takes sound to travel 1 mm


const double wheelDifRatio = DIST_THRESHHOLD / (DIST_THRESHHOLD + CHASSIS_DIAMETER);

const int turnSpeed = 3; //the speed which the vehicle turns at in mS/degrees at full speed

enum class sensorID { Front, Left, Angled };  //e number to specify which ultrasonic sensor
enum class Command { Forwards, Backwards, Left, Right, Grab, Release, Left45, Right45, TrimLeft, TrimRight, Stop };  //e number to store commands from user
enum class State { Fwd, SlightR, SlightL, R90, L90, L45, R45, R135, UTurn, SlightFwd, DoNotChange, BadValue }; //e number to define the current state of the robot

/*
 ===Bluetooth===
 HC06    ARDUINO
 TX  <-->  10 (RX)
 RX  <-->  11 (TX)

 ===Ultrasonic===
 FORWARD   ARDUINO
 Echo  <-->  7
 Trigger <-->  12

 ANGLED
 Echo  <-->  4
 Trigger <-->  A0

 LEFT
 Echo  <-->  8
 Trigger <-->  A1

 ===Motors===
 LEFT MOTOR  ARDUINO
 1 <-->  3
 2 <-->  5

 RIGHT MOTOR ARDUINO
 1 <-->  6
 2 <-->  9

 ===LEDs===
 RED <--> 13
 GREEN <--> 2

 ==Other==
 BUTTON <--> A3
 BUZZER <--> A2

 COMPASS
 SCL <--> SCL (A5)
 SDA <---> SDA (A4)

*/

//HARDWARE CONNECTIONS//
//--------------------//
//motor connections must be pwm
#define LEFT_MOTOR_1 6  
#define LEFT_MOTOR_2 9
#define RIGHT_MOTOR_1 3 
#define RIGHT_MOTOR_2 5

//ultrasonic connections and parameters
#define TRIGGER_PIN_FORWARD 12
#define TRIGGER_PIN_ANGLED A0
#define TRIGGER_PIN_LEFT A1
#define ECHO_PIN_FORWARD 7
#define ECHO_PIN_ANGLED 4
#define ECHO_PIN_LEFT 8
#define MAX_DISTANCE 2000

#define SORTWARE_SERIAL_RX 0
#define SORTWARE_SERIAL_TX 1

#define LED_RED 13
#define LED_BLUE 2

#define BUZZER A2


//front failsafe distance
#define MIN_FAILSAFE_DIST 50
//#define NO_FAILSAFE

#define LEFT_DIST_THRESHHOLD 50
#define FRONT_DIST_THRESHHOLD 30
#define ANGLED_DIST_THRESHHOLD 1

#endif
