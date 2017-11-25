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

#include "Config.h"
#include "Movement.h"
#include "Detection.h"
#include "HumanInterface.h"

HumanInterface HuI(SORTWARE_SERIAL_RX, SORTWARE_SERIAL_TX); //BT rx,tx
Movement drive(LEFT_MOTOR_1, LEFT_MOTOR_2, RIGHT_MOTOR_1, RIGHT_MOTOR_2);
Detection detection(TRIGGER_PIN_FORWARD, TRIGGER_PIN_ANGLED, TRIGGER_PIN_LEFT, ECHO_PIN_FORWARD, ECHO_PIN_ANGLED, ECHO_PIN_LEFT, MAX_DISTANCE);

Command input;  //stores human inputs

Command prevInput;

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

  int leftWheelPower = MAX_POWER;
  int rightWheelPower = MAX_POWER;

  
void changeTrim(bool left) {
  if (left) {
    rightWheelPower < 255 ? rightWheelPower++ : leftWheelPower--;
    }
  else
    leftWheelPower < 255 ? leftWheelPower++ : rightWheelPower--;
  }

void manual() {
  Serial.println("Manual");


  detection.resetDistArray(); //resets the array of previous distance readings

	while (!HuI.checkBT()) {} //wait for the first command
  
	input = HuI.getInput(); //store input

  bool forwards = true;
 

	while (input != Command::Release) { //stay in manual mode until the Release command is recieved
		switch (input) {
      
		case Command::Forwards:
      forwards = true;
			#ifndef NO_FAILSAFE 
			if (failSafeCheck()) {   //no wall to close in front
				digitalWrite(LED_RED, LOW);
				drive.drive(leftWheelPower, rightWheelPower);
			}
          
			else {                  //failsafe prohibits forward motion
				digitalWrite(LED_RED, HIGH);
				drive.stop_movement();
				delay(15);
			}
     #endif
     
     #ifdef NO_FAILSAFE
     drive.drive(leftWheelPower, rightWheelPower);
     #endif
		 break;
      
		case Command::Left:
      if (forwards) { drive.drive(0, rightWheelPower); } 
      else { drive.drive(0, -rightWheelPower); }
			break;
      
		case Command::Right:
      if (forwards) { drive.drive(leftWheelPower, 0); } 
      else { drive.drive(-leftWheelPower, 0); }
			break;
      
		case Command::Backwards:
      forwards = false;
			drive.drive(-leftWheelPower, -rightWheelPower);
     break;

    case Command::Right45:
      drive.turn(true, 45);
      drive.drive(0, 0);
      break;
      
    case Command::Left45:
      drive.turn(true, -45);
      drive.drive(0, 0);
      break;
      
    case Command::TrimLeft:
      changeTrim(true);
      break;
      
    case Command::TrimRight:
      changeTrim(false);
      break;
      
		default:
			drive.stop_movement();
		}


    if (input != Command::TrimLeft && input != Command::TrimRight) {
		//check for new input
		while (!HuI.checkBT()) { }
    prevInput = input;
    input = HuI.getInput(); 

    } else  {

    if (HuI.checkBT()) {
       input = HuI.getInput();        
      } else { input = prevInput; }
      
    }
	}
}



void L90() {
  drive.turn(true, -90);

}

void R90() {
  drive.turn(true, 90);

}

void L45() {
  drive.turn(true, -45); 

}

void R45() {
  drive.turn(true, 45);

}

void R135() {
  drive.turn(true, 135);

}

void UTurn() {
  drive.turn(true, 180);

}

bool goToTurn() {

    for (int i = 0; i < 20; i++) {

    int leftDistance = detection.getDistance(sensorID::Left);
    Serial.print("Left Distance: ");
    Serial.println(leftDistance);
    delay(30); 
    int frontDistance = detection.getDistance(sensorID::Front);
    Serial.print("Front Distance: ");
    Serial.println(frontDistance);
    delay(30);

    if (leftDistance < 50) { return false; }
    if (frontDistance < 20) { return true; }
    if (i == 19) { Serial.println("DONEDONEDONEDONEDONEDONEDONEDONEDONEDONEDONEDONEDONEDONEDONEDONEDONEDONEDONEDONEDONEDONEDONEDONEDONEDONEDONEDONEDONEDONEDONEDONEDONEDONEDONEDONE"); return true; }

    }
}




void automatic() {

  delay(100);
  
	Serial.println("start auto");

  State currentState = State::Fwd;

  detection.resetDistArray();

  State stateArray[2][2][2];

  // If the wall is too close, value = 1;
  //         F  D  L
  stateArray[0][0][0] = State::L45;
  stateArray[0][0][1] = State::DoNotChange;
  stateArray[0][1][0] = State::DoNotChange;
  stateArray[0][1][1] = State::R45;
  stateArray[1][0][0] = State::L90;
  stateArray[1][0][1] = State::R90;
  stateArray[1][1][0] = State::DoNotChange;
  stateArray[1][1][1] = State::R90;



  int leftIndex, frontIndex, angledIndex;
  
  float error, tempPower;
 
	while (input != Command::Grab) {  //stay in auto mode until the Grab command is recieved

	  //check for new input
		if (HuI.checkBT()) {
			input = HuI.getInput();
		}
	

 
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
    if (leftDistance < MIN_DISTANCE_TO_WALL) currentState = State::SlightR;
    else if (leftDistance > MAX_DISTANCE_TO_WALL) currentState = State::SlightL;
    else currentState = State::Fwd;

    // Sets distance of walls into a "too close" or "far away" in the form of 1 / 0
    leftDistance > LEFT_DIST_THRESHHOLD ? leftIndex = 0 : leftIndex = 1;
    frontDistance > FRONT_DIST_THRESHHOLD ? frontIndex = 0 : frontIndex = 1;
    angledDistance > ANGLED_DIST_THRESHHOLD ? angledIndex = 0 : angledIndex = 1;

    // looks up state on table

    if (stateArray[frontIndex][angledIndex][leftIndex] != State::DoNotChange) currentState = stateArray[frontIndex][angledIndex][leftIndex];

   // if (leftDistance > 60) currentState = State::BadValue;

    switch (currentState) {

      case State::Fwd:
        drive.drive(AUTO_POWER);
        Serial.println("FORWARD");
        break;

      case State::SlightL:
        
        error = abs(leftDistance - MAX_DISTANCE_TO_WALL);
        tempPower = AUTO_POWER * (1-(error*4/100));
        drive.drive(tempPower, AUTO_POWER);
        
        Serial.println("SLIGHT LEFT");
        Serial.println(error);
        Serial.println(tempPower);
        break;

      case State::SlightR:
        
        error = abs(leftDistance - MIN_DISTANCE_TO_WALL);
        tempPower = AUTO_POWER * (1-(error*4/100));
        drive.drive(AUTO_POWER, tempPower);
        
        Serial.println("SLIGHT RIGHT");
        Serial.println(error);
        Serial.println(tempPower);
        break;

      case State::L90:
        Serial.println("LEFT 90");
        if (goToTurn()) { L90(); }
        detection.resetDistArray();
        break;

      case State::R90:
        Serial.println("RIGHT 90");
        goToTurn();
        R90();
        detection.resetDistArray();
        break;

      case State::L45:
        Serial.println("LEFT 45");
        if (goToTurn()) { L45(); }
        detection.resetDistArray();
        int tempAngDist;
        tempAngDist = detection.getDistance(sensorID::Angled);
        if (tempAngDist == 12 || tempAngDist - 1 == 12 || tempAngDist + 1 == 12) {
          goToTurn();
          R135();
          detection.resetDistArray();

        }
        break;

      case State::R45:
        Serial.println("Right 45");
        goToTurn();
        R45();
        detection.resetDistArray();
        break;

      case State::R135:
        Serial.println("RIGHT 135");
        goToTurn();
        R135();
        detection.resetDistArray();
        break;

      case State::UTurn:
        Serial.println("UTURN");
        UTurn();
        detection.resetDistArray();
        break;

     case State::BadValue:
        Serial.println("BadValue");
        drive.stop_movement();
        Serial.print("Left Distance: ");
        for (int i = 0; i < 10; i++) {
        int leftDistance = detection.getDistance(sensorID::Left);
        Serial.print(leftDistance);
        Serial.print(", ");
       delay(100); 

        }
        delay(20000);  
        
        break;

    }
  }
}

void loop() {
	//robot starts in manual mode and waits for a command. Whenever it is told to leave a mode, it switches to the other one
  /*
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
  */
	manual();
	automatic();

}
