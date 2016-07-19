// Door.h

// Function Coding for tracking of Door States
// 2016 V1 By Sean Zeng

#ifndef _DOOR_h
#define _DOOR_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <Servo.h>

class Door {
public:
	Door(Servo doorServo, int doorPin1, int relayPin1, int closeAngle1, int openAngle1);

	// Set Door States Manually
	void setBoltLocked();
	void setBoltOpen();
	void setDoorOpen();
	void setDoorClosed();
	void setDoorStayOpen();
	void setDoorStayClosed();

	// Operate Bolt
	void lockdoor();
	void unlockdoor();

	// Check Door States
	bool isDoorOpen();
	bool isDoorStayOpen();
	bool isBoltUnlocked();

	// Check last Operation State
	bool checkSuccess();
	String errorReason();
	bool removeError();

	// Add Pin for status reporting
	void addActivityPin(int pin);

private:
	// Variables to check door state
	bool doorIsOpen;
	bool doorStayOpen;
	bool boltUnlocked;

	// Pin Numbers
	int doorPin;
	int relayPin;
	
	// Servo Angles to Servo.write(angle)
	int closeAngle;
	int openAngle;

	// Check if last operation completed sucessfully (e.g. bolt was locked but door was not closed)
	bool error;
	String stringErrorReason;

	// Servo Class to control servo
	Servo servoDoor;

	// Set Pin State for status reporting
	void activityPin(bool onOff);
	int activityPinNum;
	bool activityPinSet = false;

};

#endif

