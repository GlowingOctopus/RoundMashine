#include "HumanInterface.h"
#include "Config.h"

//check if a new command is available over BT
bool HumanInterface::checkBT() {
	if (Serial.available() > 0) { //MODE_BUTTON
		return true;
	}
	else {
		return false;
	}
}

//translate a command and return it
Command HumanInterface::getInput() {

	// try catch this line to see if button has been pressed
	char key = Serial.read();
	Serial.print('R'); //Recieved: 
	Serial.println(key);

	Command input;
	switch (key) {
	case 'w': // old: 8
		input = Command::Forwards;
		Serial.println("unput = forwards");
		break;
	case 's': // old: 2
		input = Command::Backwards;
		break;
	case 'a':// old: 4
		input = Command::Left;
		break;
	case 'd': // old: 6
		input = Command::Right;
		break;
	case 'r': // old: D
		input = Command::Release;
		break;
	case 'g': // old: C
		input = Command::Grab;
		break;
	case 'q':
		input = Command::Left45;
		break;
	case 'e':
		input = Command::Right45;
		break;
	case 'z':
		input = Command::TrimLeft;
		break;
	case 'x':
		input = Command::TrimRight;
		break;
	default:
		input = Command::Stop;
	}
	return input;
}

//HumanInterface constructor. It starts the serial port for the BT
HumanInterface::HumanInterface(int rx, int tx) {
}
