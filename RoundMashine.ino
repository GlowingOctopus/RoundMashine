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
 * Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
 * pullup resistors on I2C for compass?
 *
 * void turn(intensity) should take a speed to turn at (implemented by ratio between wheel speeds) and be used for straight wall following. speed is determined by error between wanted value and measured value
 * void turn(angle) should be called when a wall disappears on the side or appears in front
 *
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
#define RIGHT_MOTOR_1 5 
#define RIGHT_MOTOR_2 3

//ultrasonic connections and parameters
#define TRIGGER_PIN_FORWARD 12
#define TRIGGER_PIN_ANGLED A0
#define TRIGGER_PIN_LEFT A1
#define ECHO_PIN_FORWARD 7
#define ECHO_PIN_ANGLED 4
#define ECHO_PIN_LEFT 8
#define MAX_DISTANCE 200

#define SORTWARE_SERIAL_RX 0
#define SORTWARE_SERIAL_TX 1

#define LED_RED 13
#define LED_BLUE 2

#define BUZZER A2


//front failsafe distance
#define MIN_FAILSAFE_DIST 5
#define NO_FAILSAFE

#define DIST_THRESHHOLD 3


#include "RoundMachineClasses.h"

HumanInterface HuI(SORTWARE_SERIAL_RX, SORTWARE_SERIAL_TX); //BT rx,tx
Movement drive(LEFT_MOTOR_1, LEFT_MOTOR_2, RIGHT_MOTOR_1, RIGHT_MOTOR_2);
Detection detection(TRIGGER_PIN_FORWARD, TRIGGER_PIN_ANGLED, TRIGGER_PIN_LEFT, ECHO_PIN_FORWARD, ECHO_PIN_ANGLED, ECHO_PIN_LEFT, MAX_DISTANCE);

Command input;  //stores human inputs

#define DEBUGGING

void setup() {
  Serial.begin(115200);
 /* Serial.println("Connection Established."); 
  Serial.println(detection.getDistance(sensorID::Front));
  Serial.println(detection.getDistance(sensorID::Angled));
  Serial.println(detection.getDistance(sensorID::Left)); */
	pinMode(LED_RED, OUTPUT);

 

}

// failSafeCheck() returns false if the wall in front of the robot is closer than MIN_FAILSAFE_DIST
bool failSafeCheck() {
	if (detection.getDistance(sensorID::Front) < MIN_FAILSAFE_DIST) return false;
	else return true;
}

void manual() {
  Serial.println("Manual");

	while (!HuI.checkBT()) {} //wait for the first command
  
	input = HuI.getInput(); //store input

	while (input != Command::Release) { //stay in manual mode until the Release command is recieved
		switch (input) {
      
		case Command::Forwards:
			#ifndef NO_FAILSAFE 
			if (failSafeCheck()) {   //no wall to close in front
				digitalWrite(LED_RED, LOW);
				drive.drive(MAX_POWER);
			}
          
			else {                  //failsafe prohibits forward motion
				digitalWrite(LED_RED, HIGH);
				drive.stop_movement();
				delay(15);
			}
     #endif
     #ifdef NO_FAILSAFE
    // Serial.println("drive!");
     drive.drive(MAX_POWER);
     #endif
			break;
		case Command::Left:
			drive.turn(true, -90);
			break;
		case Command::Right:
			drive.turn(true, 90);
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



void L90() {
  drive.turn(true, -90);

}

void R90() {
  drive.turn(false, 90);

}

void L45() {
  drive.turn(false, -45); 

}

void R45() {
  drive.turn(false, 45);

}

void R135() {
  drive.turn(false, 135);

}

void UTurn() {
  drive.turn(true, 180);

}







void automatic() {
	Serial.println("start auto");

 State currentState = State::Fwd;

State stateArray[2][2][2];


  // If the wall is too close, value = 1;
  //         F  D  L
  stateArray[0][0][0] = State::L45;
  stateArray[0][0][1] = State::Fwd;
  stateArray[0][1][0] = State::Fwd;
  stateArray[0][1][1] = State::L90;
  stateArray[1][0][0] = State::R45;
  stateArray[1][0][1] = State::Fwd;
  stateArray[1][1][0] = State::Fwd;
  stateArray[1][1][1] = State::R90;



  int leftIndex, frontIndex, angledIndex;


 
 
	while (input != Command::Grab) {  //stay in auto mode until the Grab command is recieved

	  //check for new input
		if (HuI.checkBT()) {
			input = HuI.getInput();
		}
	

 
    delay(1000);
    //take readings from sensors
    int leftDistance = detection.getDistance(sensorID::Left);
    Serial.print("Left Distance: ");
    Serial.println(leftDistance);
    delay(30); 
    int frontDistance = detection.getDistance(sensorID::Front);
    Serial.print("Front Distance: ");
    Serial.println(frontDistance);
    delay(30);
    int angledDistance = detection.getDistance(sensorID::Angled);
    Serial.print("Angled Distance: ");   
    Serial.println(angledDistance);
    delay(30);


    // puts into a slight right or left turn state if in need of adjustment
    if (leftDistance < 2) currentState == State::SlightR;
    else if (leftDistance > 2 && leftDistance < 4) currentState == State::SlightL;

    // Sets distance of walls into a "too close" or "far away" in the form of 1 / 0
    if (leftDistance > 4) leftIndex = 0; else leftIndex = 1;
    if (frontDistance > 4) frontIndex = 0; else frontIndex = 1;
    if (angledDistance > 4) angledIndex = 0; else angledIndex = 1;

    // looks up state on table
    currentState = stateArray[frontIndex][angledIndex][leftIndex];


    // Edge case - happens at the end for maximum override
    if ((angledDistance == 20 || angledDistance + 1 == 20 || angledDistance - 1 == 20) && (frontDistance == 20 || frontDistance + 1 == 20 || frontDistance - 1 == 20)) {
       currentState = State::UTurn;

    }

    switch (currentState) {

      case State::Fwd:
        drive.drive(MAX_POWER);
        Serial.println("FORWARD");
        break;

      case State::SlightL:
        drive.drive(ADJUST_POWER, MAX_POWER);
        Serial.println("SLIGHT LEFT");
        break;

      case State::SlightR:
        drive.drive(MAX_POWER, ADJUST_POWER);
        Serial.println("SLIGHT RIGHT");
        break;

      case State::L90:
        Serial.println("LEFT 90");
        L90();
        break;

      case State::R90:
        Serial.println("RIGHT 90");
        R90();
        break;

      case State::L45:
        Serial.println("LEFT 45");
        L45();
        int tempAngDist;
        tempAngDist = detection.getDistance(sensorID::Angled);
        if (tempAngDist == 20 || tempAngDist - 1 == 20 || tempAngDist + 1 == 20) {
          R135();

        }
        break;

      case State::R45:
        Serial.println("Right 45");
        R45();
        break;

      case State::R135:
        Serial.println("RIGHT 135");
        R135();
        break;

      case State::UTurn:
        Serial.println("UTURN");
        UTurn();
        break;

}


    
    






/*
    if (leftDistance > DISTANCE_TO_WALL && leftDistance < DISTANCE_TO_WALL + 2) { // a little too far away from left wall
      drive.turn(false, -(leftDistance - DISTANCE_TO_WALL) * 2);  //correct a little to the left
    }
    else if (leftDistance > DISTANCE_TO_WALL) { //no wall to the left
      drive.turn(false, -45);                   //turn around the left corner 45 degrees
    }
      
    else if (leftDistance < DISTANCE_TO_WALL) { //too close to left wall
      drive.turn(false, -(leftDistance - DISTANCE_TO_WALL) * 2);   //correct a little to the right
    }
      
    if (frontDistance < DISTANCE_TO_WALL || angledDistance < DISTANCE_TO_WALL) {    // come to a road block at the front, must turn right
      drive.stop_movement();
      drive.turn(true, 45);   //turn on spot 45 degrees right
    }
  
    drive.drive(MAX_POWER);  //keep driving

  //check for new input
  if (HuI.checkBT()) {
    input = HuI.getInput();
  }*/
  }
}

void loop() {
	//robot starts in manual mode and waits for a command. Whenever it is told to leave a mode, it switches to the other one
	manual();
	automatic();

}
