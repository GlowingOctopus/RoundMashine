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
	case 0x38: // 8
		input = Command::Forwards;
		Serial.println("unput = forwards");
		break;
	case 0x32: // 2
		input = Command::Backwards;
		break;
	case 0x34:// 4
		input = Command::Left;
		break;
	case 0x36: // 6
		input = Command::Right;
		break;
	case 0x44: // D
		input = Command::Release;
		break;
	case 0x43: // C
		input = Command::Grab;
		break;
	case 0x45: // E
		input = Command::RLeft;
		break;
	case 0x46: // F
		input = Command::RRight;
		break;
	default:
		input = Command::Stop;
	}
	return input;
}

//HumanInterface constructor. It starts the serial port for the BT
HumanInterface::HumanInterface(int rx, int tx) {
}