///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////// WELCOME TO ROUNDMASHINE /////////////////////
///////////////////// ----- Pat and Zac ----- /////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

/* NOTES
 * block units
 * sonar smoothing/averaging
 * add overid button to switch to auto without BT
 */

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
Trigger <-->  12

LEFT
Echo  <-->  8
Trigger <-->  12

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

COMPASS
SCL <--> SCL (A5)
SDA <---> SDA (A4)

*/

//HARDWARE CONNECTIONS//
//--------------------//
//motor connections must be pwm
#define LEFT_MOTOR_1 3  
#define LEFT_MOTOR_2 5
#define RIGHT_MOTOR_1 6 
#define RIGHT_MOTOR_2 9

//ultrasonic connections and parameters
#define TRIGGER_PIN_FORWARD 12
#define TRIGGER_PIN_ANGLED A0
#define TRIGGER_PIN_LEFT A1
#define ECHO_PIN_FORWARD 7
#define ECHO_PIN_ANGLED 4
#define ECHO_PIN_LEFT 8
#define MAX_DISTANCE 200

#define SORTWARE_SERIAL_RX 10
#define SORTWARE_SERIAL_TX 11

#define LED_RED 13
#define LED_BLUE 2

//front failsafe distance
#define MIN_FAILSAFE_DIST 5

#include "RoundMachineClasses.h"

HumanInterface HuI(SORTWARE_SERIAL_RX, SORTWARE_SERIAL_TX); //BT rx,tx
Movement drive(LEFT_MOTOR_1, LEFT_MOTOR_2, RIGHT_MOTOR_1, RIGHT_MOTOR_2); 
Detection detection(TRIGGER_PIN_FORWARD, TRIGGER_PIN_ANGLED, TRIGGER_PIN_LEFT, ECHO_PIN_FORWARD, ECHO_PIN_ANGLED, ECHO_PIN_LEFT, MAX_DISTANCE); 

Command input;  //stores human inputs

void setup() {  
  
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  Serial.print("Setup");
  //Serial.println(Movement::Compass.GetHeadingDegrees());
}

bool failSafeCheck() {
  // if wall is too close return false      OLD---[and robot is being driven towards it, STOP (or reverse wheels depending on testing)]---
  if (detection.getDistance(sensorID::Front) < MIN_FAILSAFE_DIST){
    return false;
    }
    else return true;
  }

void manual() {
  //Serial.println("start manual");
  while (!HuI.checkBT()) {} //wait for a command

  input = HuI.getInput(); //store input
  
  while (input != Command::Release) { //stay in manual mode until the Release command is recieved
    switch (input) {
        case Command::Forwards:
            if (failSafeCheck()){
              digitalWrite(13, LOW);
              drive.drive(100);
            }
            else {
              digitalWrite(13, HIGH);
              drive.stop_movement();
              delay(15);
            }
            break;
        case Command::Left:
          drive.turn(true, -20);
          break;
        case Command::Right:
          drive.turn(true, 20);
          break;
        default:
          drive.stop_movement();
      }

    //check for new input
    if (HuI.checkBT()) {
      input = HuI.getInput();
    }
  }
}

void automatic() {
  //Serial.println("start auto");
  while (input != Command::Grab) {  //stay in auto mode until the Grab command is recieved

    //take readings from sensors
    int leftDistance = detection.getDistance(sensorID::Left); 
    int frontDistance = detection.getDistance(sensorID::Front);
    int angledDistance = detection.getDistance(sensorID::Angled);
    
    if (leftDistance > DISTANCE_TO_WALL && leftDistance < DISTANCE_TO_WALL + 2) { // a little too far away
      drive.turn(false, -(leftDistance - DISTANCE_TO_WALL) * 2);
    }
    else if (leftDistance > DISTANCE_TO_WALL) { //no wall to the left
      drive.turn(false, -45);
    }
      
    else if (leftDistance < DISTANCE_TO_WALL) { //too close to left wall
      drive.turn(true, -(leftDistance - DISTANCE_TO_WALL) * 2);
    }
      
    if (frontDistance < DISTANCE_TO_WALL || angledDistance < DISTANCE_TO_WALL) {
      drive.stop_movement();
      drive.turn(true, 45);
    }
  
    drive.drive(MAX_POWER);

  //check for new input
  if (HuI.checkBT()) {
    input = HuI.getInput();
  }
  }
}

void loop() {
  Serial.print("Start.\nonSpot = true, right 90d\n");
  drive.turn(true, 90);
  delay(1000);
  Serial.print("onSpot = true, left 90d");
  drive.turn(true, -90);
  delay(1000);
  drive.turn(false, 90);
  delay(1000);
  drive.turn(false, -90);
  delay(1000);
  //manual();
  //automatic();
}
