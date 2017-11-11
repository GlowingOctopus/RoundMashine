#ifndef HUMAN_INTERFACE_H
#define HUMAN_INTERFACE_H

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