// 
// Function Coding for tracking of Door States
// 2016 V1 By Sean Zeng
// 

#include "Door.h"

// Initalzation of Class
Door::Door(Servo doorServo, int doorPin1, int relayPin1, int closeAngle1, int openAngle1) {
	// Intialization Treats door as unlocked
	// Must be set locked in main code;
	doorIsOpen = false;
	doorStayOpen = false;
	boltUnlocked = true;

	error = false;
	stringErrorReason = "None";

	doorPin = doorPin1;
	relayPin = relayPin1;

	closeAngle = closeAngle1;
	openAngle = openAngle1;

	servoDoor = doorServo;

}

// Set Door States Manually
void Door::setBoltLocked() {
	boltUnlocked = false;
}
void Door::setBoltOpen() {
	boltUnlocked = true;
}
void Door::setDoorOpen() {
	doorIsOpen = true;
}
void Door::setDoorClosed() {
	doorIsOpen = false;
}
void Door::setDoorStayOpen() {
	Serial.println("Door autolock disabled");
	doorStayOpen = true;
}
void Door::setDoorStayClosed() {
	Serial.println("Door autolock enabled");
	doorStayOpen = false;
}

// Operate Bolt
void Door::lockdoor() {

	activityPin(1);

	Serial.print("Sending Servo angle: ");
	Serial.print(closeAngle);
	Serial.println(" To LOCK Door...");

	// Operate relay to power Servo, then move it
	digitalWrite(relayPin, HIGH);
	delay(10);

	servoDoor.write(closeAngle);
	delay(1000);

	digitalWrite(relayPin, LOW);

	// Save State
	boltUnlocked = false;
	
	Serial.println("DONE!");

	activityPin(0);

}
void Door::unlockdoor() {

	activityPin(1);

	Serial.print("Sending Servo angle: ");
	Serial.print(openAngle);
	Serial.println(" To OPEN Door...");
	
	// Operate relay to power Servo, then move it
	digitalWrite(relayPin, HIGH);
	delay(10);

	servoDoor.write(openAngle);
	delay(1000);

	digitalWrite(relayPin, LOW);

	// Save State
	boltUnlocked = true;

	Serial.println("DONE!");

	// If error from locking door was there, remove it
	if (error = true) {
		error = false;
		stringErrorReason = "";
	}

	activityPin(0);
}

// Check Door States
bool Door::isDoorOpen() {

	if (digitalRead(doorPin) == HIGH) {
		doorIsOpen = false;
	}
	else {
		doorIsOpen = true;
	}

	return doorIsOpen;
};
bool Door::isDoorStayOpen() {
	return doorStayOpen;
}
bool Door::isBoltUnlocked() {
	return boltUnlocked;
}

// Check last Operation State
bool Door::checkSuccess() {

	// Check for one error
	if (isDoorOpen() && !isBoltUnlocked()) {
		error = true;
		stringErrorReason = "ERROR: Door Open, Should be locked";
	}

	return error;
}
String Door::errorReason() {
	return stringErrorReason;
}
bool Door::removeError() {

	if (error == true) {
		stringErrorReason = "";
		error = false;
		return true; // Error was removed
	}

	return false; // no errors to remove
}

void Door::addActivityPin(int pin) {
	activityPinNum = pin;
	activityPinSet = true;
}

void Door::activityPin(bool onOff) {
	if (activityPinSet == true) {
		if (onOff == 1) {
			digitalWrite(activityPinNum, HIGH);
		} else {
			digitalWrite(activityPinNum, LOW);
		}
	}
	else {
		//Do nothing
	}


}


