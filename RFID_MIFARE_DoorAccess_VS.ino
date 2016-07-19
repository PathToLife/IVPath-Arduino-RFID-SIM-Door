/******************************************
PURPOSE:  To Securely Control Access to a Door
Created by: Sean Zeng from New Zealand
DATE:   2/2014
*******************************************/

/*
* This sketch uses the MFRC522 Library to use ARDUINO RFID MODULE KIT 13.56 MHZ WITH TAGS SPI W AND R BY COOQROBOT.
* The library file MFRC522.h has a wealth of useful info. Please read it.
* The functions are documented in MFRC522.cpp.
*
* Based on code Dr.Leong   ( WWW.B2CQSHOP.COM )
* Created by Miguel Balboa (circuitito.com), Jan, 2012.
* Rewritten by Søren Thing Andersen (access.thing.dk), fall of 2013 (Translation to English, refactored, comments, anti collision, cascade levels.)
*
* This library has been released into the public domain.
*/

/*
Not all pins on the Mega and Mega 2560 support change interrupts, so only the following can be used for RX:
10, 11, 12, 13, 14, 15, 50, 51, 52, 53, A8(62), A9(63), A10(64), A11(65), A12(66), A13(67), A14(68), A15(69).
*/

/* SIM Commands Read from Data Sheet. Sean Zeng =)

AT&W Saves the sim Settings

See Module Model Information
AT+ATI

Signal Quality
AT+CSQ
+CSQ: 31,0
<rssi><recieved signal strength indication>, <ber>=<bit error rate>
rssi corellated with db
0 = -115db or less
2-30 = -110 to -54 db
31 = -52 db or greater
99 = Undetectable

Sim is registerd?
AT+CREG?
+CREG: 0,1

Enable More Error code info
AT+CMEE=1

Check SIM PIN
AT+CPIN?
OK

Baud Rate of Serial Communication
AT+IPR?
0 is auto detect but i doesn't work. More like very high baud only

See own number
AT+CNUM

Memory Used:
AT+CPMS?

Memory types avalible:
AT+CPMS=?

Set Memory Storage to on sim and phone memory
AT+CPMS="MT"
+CPMS: <used_space>,<max_space>

/// SMS  ///

Set Text Message bits 1,0 via int
AT+CSMP=?
AT+CSMP=17,167,0,0 <- 17 no idea, 167 text expires in 24 hours, 0, 0 msg 7 bit text

Character Format GSM, UCS2, HEX, IRA, PCCP, 8859-1
AT+CSCS=Hex <- Unicode
AT+CSCS?
AT+CSCS="GSM"

SMS Mesage Format Sets Text Mode
AT+CMGF=1 <- Displays MSG as TEXT, Failed text may reset this
0 is PDU Mode

Send Text
AT+CMGS="0279465037"
> Text
End with char(26) (Ctrl-Z), so we do Serial1.write(26)

Send Text From storage
AT-CMSS

Write Text to storage
AT-CMGW

New SMS
AT+CNMI

Recieve Msg's
+CMTI: "SM",5

Save SMS Settings
AT+CSAS

Delete Text Msg from memory
AT+CMGD=number
AT+CMGD=number,1 deletes all read msg, send untouched
2 deletes all read and sent
3 deletes all read, send all deleted
4 deletes all inclu unread

Read List of Msgs
AT+CMGL=?
AT+CMGL="REC UNREAD",0 Show ALL Unread
AT+CMGL="STO SENT",0 Sent msgs
AT+CMGL="ALL", Show ALL MSG
+CMGL: <index>,<status>,<from_address>,<mr>,<scts><CRLF><data>
index	The memory index number, use this index to read or delete this message.
status	The status of this message. For received messages this can be "REC READ" or "REC UNREAD" depending on whether the messages has been read or listed before.
from_address	The subscriber number of the person who sent the message.
mr	The reference number of this message. Most modems keep this field empty.
scts	The time the message was forwarded to this phone or modem.
data	The actual message data in plain text

Read Text
AT+CMGR


/// DTMF Tones (number communication by ear) ///


*/

/* RFID uses SPI Communication bus
This is MOSI, MISO, SCLK, SDA
With 3.3V GND
With RST
And Redundant IRQ (Interrupt request pin)

Arduino 2560 Matching pins:

--Special--
49 RST (You choose this)

--SPI--
MISO: 50
MOSI: 51
Clock SCK: 52
Slave Select SDA: 53
*/

#include "Door.h"
#include <SPI.h>//include the SPI bus library
#include <MFRC522.h>//include the RFID reader library
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <stdio.h>
#include <DS1302.h>
#include <DHT.h>

// RFID PINS
#define SS_PIN 53  // slave select pin
#define RST_PIN 49  // RFID reset pin

// DOOR PINS
#define SERVO_PWM 7
#define RELAY_PIN 6
#define DOOR_PIN 5


// Interfaces IN OUT
#define BUTTON_OPENCLOSE 4
#define STAYOPEN_PIN 13
#define OPENLIGHT_PIN 12
#define ALARM_PIN 11

#define DOOR_MOTOR_IN_OPERATION 10
#define LED 

// Turn this to off so the led wpn't be constantly on
#define LED_ARDUINO_L 13

// SERVO Door Angles
#define SERVO_CLOSEANGLE 10
#define SERVO_OPENANGLE 135

// Digital Clock Pin
#define DHT_PIN 3

// Set the appropriate digital I/O pin connections.
// Data sheet: http://datasheets.maximintegrated.com/en/ds/DS1302.pdf
#define rtcCePin 10  // RTC Chip Enable
#define rtcIoPin 11  // RTC Input/Output
#define rtcSclkPin 12  // RTC Serial Clock

// Instatiate a MFRC522 reader object.
MFRC522 mfrc522(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key;//create a MIFARE_Key struct named 'key', which will hold the card information

// Correct Passphrase from Card: Block 61 "Goodluck:)_____"
int cardReadBlockNumber = 61;
// If need to print in serial, use Serial.write(byte[j]); Since ASCII numbers need translating
const byte cardReadBlockContent[16] = "Goodluck:)_____"; 
const byte cardReadBlockContentA[16] = "thyetgjshtgdjwh";

int cardWriteBlockNumber = 50;
byte writeToBlock[16] = "728ZestApart___";

byte readBackBlock[18]; //This array is used for reading out a block. The MIFARE_Read method requires a buffer that is at least 18 bytes to hold the 16 bytes of a block.

// Control Variables for SimpleMultiState Operation of main loop
char globalByte[9]; //For serial return of information
bool writeToCardTrue = false;
bool cardDumpTrue = false;
char globalChar[4]; // For Servo control across modules
int servoDoorAngle = 10;

// Millis Multi States
unsigned long previous500msMillis = 0;
unsigned long previousButtonMillis = 0;
unsigned long buttonPressTime = 0;
unsigned long previousDoorActionMillis = 0;

// Initiate Class for handling servo states
Servo servoDoor;
// Intitiate Class for handling the door, relays and reading state
// Also handles errors, such as door open and trying to lock (It will still operate, just let u know there is a error)
Door door1(servoDoor, DOOR_PIN, RELAY_PIN, SERVO_CLOSEANGLE, SERVO_OPENANGLE);

// This enables timing of button hold down
bool countButtonPressTime = false;
// This enables the timing of doorLock loop
bool beginArmingDoorClose = false;
// This enables timing of doorOpen Alarm loop
bool beginArmingDoorOpen = false;
// This is a alarm modify prevention if other loops try to stop it
bool preventAlarmChange = false;

// LCD START
// Addr for lcd 0x3D 0x3E
// OLED 0x3C
// Big LEC 0x23

LiquidCrystal_I2C lcdA(0x3E, 16, 2);
LiquidCrystal_I2C lcdB(0x3D, 16, 2);

boolean lcdA_blink = false;
boolean lcdB_blink = false;

// Create a DS1302 object.
DS1302 rtc(rtcCePin, rtcIoPin, rtcSclkPin);

// Set DHT Digital Humidity Temperature
DHT dht(DHT_PIN, DHT11);

// LCD Custom Char for RAM storage
uint8_t smileGoate[8] = { 0x0, 0xA, 0xA, 0x0, 0x11, 0x11, 0xE, 0x4 };
uint8_t smileHair[8] = { 0x1B, 0x11, 0x0, 0xA, 0xA, 0x0, 0x11, 0xE };
uint8_t tempSymbol[8] = { 0x4, 0xA, 0xA, 0xA, 0xE, 0x1F, 0x1F, 0xE };
uint8_t humidSymbol[8] = { 0x4, 0x4, 0xA, 0xA, 0x11, 0x11, 0x11, 0xE };
uint8_t leftBoarder[8] = { 0x1F, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F };
uint8_t rightBoarder[8] = { 0x1F, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1F };

/// BEGIN ///
void setup() {
	Serial.begin(9600);        // Initialize serial communications with the PC
	SPI.begin();               // Init SPI bus
	mfrc522.PCD_Init();        // Init MFRC522 card (in case you wonder what PCD means: proximity coupling device)

	Serial.println("Scan a MIFARE Classic card");

	// Prepare the security key for the read and write functions - all six key bytes are set to 0xFF at chip delivery from the factory.
	// Since the cards in the kit are new and the keys were never defined, they are 0xFF
	// Sean: FF FF FF FF FF FF ?? == 0xFF
	// if we had a card that was programmed by someone else, we would need to know the key to be able to access it. This key would then need to be stored in 'key' instead.

	for (byte i = 0; i < 6; i++) {
		key.keyByte[i] = 0xFF;//keyByte is defined in the "MIFARE_Key" 'struct' definition in the .h file of the library
	}

	servoDoor.attach(SERVO_PWM);

	pinMode(DOOR_PIN, INPUT);
	pinMode(STAYOPEN_PIN, OUTPUT);
	pinMode(RELAY_PIN, OUTPUT);
	pinMode(BUTTON_OPENCLOSE, INPUT);
	pinMode(ALARM_PIN, OUTPUT);
	pinMode(OPENLIGHT_PIN, OUTPUT);
	pinMode(DOOR_MOTOR_IN_OPERATION, OUTPUT);

	pinMode(LED_ARDUINO_L, OUTPUT);

	digitalWrite(LED_ARDUINO_L, LOW);

	// Vital Funtions to locked door ans keep closed;
	door1.setDoorStayClosed();

	// Report door operation to led
	door1.addActivityPin(DOOR_MOTOR_IN_OPERATION);

	if (digitalRead(DOOR_PIN) == 1) {
		door1.lockdoor();
		Serial.println("Door is Closed and Locked");
	}
	else {
		door1.setDoorOpen();
		door1.setBoltOpen();
		Serial.println("Door is Open and Unlocked");
	}

	// initalize LCD
	lcdA.begin();
	// Turn On backlight and print "Hello World:
	lcdA.backlight();
	lcdA.print("Hello, World 1A");
	//lcdA.blink();

	// initalize LCD
	lcdB.begin();
	// Turn On backlight and print "Hello World:
	lcdB.backlight();
	lcdB.print("Hello, World 2B");
	//lcdB.blink();



	Serial.println("Ready for commands");
}

void loop() {

	// Millis Update
	unsigned long currentMillis = millis();

	// Serial Check, read and execute every 0.5s
	if ((currentMillis - previous500msMillis) > 500) {
		previous500msMillis = currentMillis;

		serialEvent();
		
		// Return true if problem found;
		if (doorCheckError()) { // If door was manually opened
			digitalWrite(ALARM_PIN, HIGH);
			door1.removeError();
		}

		if (door1.isDoorOpen() == true) {
			digitalWrite(OPENLIGHT_PIN, HIGH);
		}
		else
		{
			digitalWrite(OPENLIGHT_PIN, LOW);
		}
	}

	// Button To Operate Door Check
	// Will active if button was being counted for hold down time, or if button was pressed
	// Only when button is released will operations commence
	// Debounce prevents any other readings from being taken account once new time for previousButtonMillis is set
	if (digitalRead(BUTTON_OPENCLOSE) == 1 || countButtonPressTime == true) {

		// Button Debounce, if change occured less than 0.5 sec before last button press, then ignore
		if ((currentMillis - previousButtonMillis) > 500) {

			if (countButtonPressTime == false) {
				Serial.println("Button Was Pressed, Timing...");
				countButtonPressTime = true;
				buttonPressTime = millis();
			}
			// Once button is released, on of two operations will be done based upon if 3000ms had passed
			else if (digitalRead(BUTTON_OPENCLOSE) == 0) {

				Serial.print("Timed: ");
				Serial.print(currentMillis - buttonPressTime);
				Serial.println("ms");

				// Button was held for 3 sec thus toggle keep door open
				// Note this will not open door if it was closed
				// The next button press (for less than 3000ms) will take care of that
				if ((currentMillis - buttonPressTime) > 3000) {
					
					// Led should flash here, interrupts should take care of this part

					// Toggle door stay open
					if (door1.isDoorStayOpen() == false) {
						door1.setDoorStayOpen();
						digitalWrite(STAYOPEN_PIN, HIGH);
					}
					else {
						digitalWrite(STAYOPEN_PIN, LOW);
						door1.setDoorStayClosed();
					}
					
				}
				// Button was released without 3 sec passing, thus just toggle door
				else {
					// Lock or unlock door
					toggleDoor();
				}

				// Start debounce lockout
				previousButtonMillis = currentMillis;
				countButtonPressTime = false;
			}
		}
		else {
			// Debounce in effect here
			// Else Do nothing
		}
	}

	// This closes the door automatically after 2.5 sec once it is closed
	// Works only if bolt was unlocked and doorStayOpen not set
	// If door was opened for 60 sec, then alarm should buzz continuously
	if (door1.isBoltUnlocked() && !door1.isDoorStayOpen()) {

		// Flip flop loop below, switching states disables the millis of other state

		// If door is opened 
		if (door1.isDoorOpen()) {
			// Stop Close Count
			beginArmingDoorClose = false;

			// Start counting time if haven't already and if door open too long, buzz
			if (beginArmingDoorOpen == false) {
				Serial.println("Door is Open, Alarm will sound in 60s");
				previousDoorActionMillis = currentMillis;
				beginArmingDoorOpen = true;

			}
			else if ((currentMillis - previousDoorActionMillis) > 60000) {
				if (digitalRead(ALARM_PIN) == LOW) 
				{
					Serial.println("ALAM PIN SET HIGH");
					Serial.println("Door Should Be Closed, Else StayOpen Should be Set");
					digitalWrite(ALARM_PIN, HIGH);
				}
			}
		}
		// Door must be closed
		else {
			// Stop Open Count
			beginArmingDoorOpen = false;

			// If alarm was on, and ok to turn off, do so
			if (preventAlarmChange == false) {
				
				digitalWrite(ALARM_PIN, LOW);
			}

			// If count was not started, then start it and if door closed for 2.5 sec, lock it
			if (beginArmingDoorClose == false) {
				Serial.println("Door is closed, will be auto locked in 2.5s");
				previousDoorActionMillis = currentMillis;
				beginArmingDoorClose = true;
			}
			else if ((currentMillis - previousDoorActionMillis) > 2500) {
				
				door1.lockdoor();

				// Reset State so count will start again next time, note count won't start unless bolt is opened
				// Thus repeat won't happen
				beginArmingDoorClose = false;
			}
		}

	}



	// Look for new cards (in case you wonder what PICC means: proximity integrated circuit card)
	if (!mfrc522.PICC_IsNewCardPresent()) {//if PICC_IsNewCardPresent returns 1, a new card has been found and we continue

		return;//if it did not find a new card is returns a '0' and we return to the start of the loop
	}

	Serial.println("Card Detected Authenticating..");

	// Select one of the cards
	if (!mfrc522.PICC_ReadCardSerial()) {//if PICC_ReadCardSerial returns 1, the "uid" struct (see MFRC522.h lines 238-45)) contains the ID of the read card.
		Serial.println("Something Went Wrong When Reading UID");
		return;//if it returns a '0' something went wrong and we return to the start of the loop
	}

	Serial.print("Card ID: ");
	Serial.print(mfrc522.uid.size, DEC);
	for (int i = 0; i < (int)mfrc522.uid.size; ++i) {
		Serial.print(mfrc522.uid.uidByte[i], HEX);
	}
	Serial.println();

	if (writeToCardTrue) {
		writeBlock(cardWriteBlockNumber, writeToBlock);
	}

	if (cardDumpTrue) {

		Serial.println("Block   0  1  2  3   4  5  6  7   8  9 10 11  12 13 14 15");

		for (int i = 63; i >= 0; --i) {
			Serial.print("  ");
			Serial.print(i);
			Serial.print("   ");
			readBlock(i, readBackBlock);
			for (int i = 0; i < 16; ++i) {
				Serial.write(readBackBlock[i]); // Output as Byte to ASCII
			};
			Serial.println();
		}

		// Read out all information
		mfrc522.PICC_DumpToSerial(&(mfrc522.uid));
	}

	// Authenticate
	Serial.println("Reading...");

	readBlock(cardReadBlockNumber, readBackBlock);

	if (checkBytesIdentical(readBackBlock, cardReadBlockContent, 10)) {
		Serial.println("Card Authenticated");
		
		toggleDoor();
		
	}
	else {
		Serial.println("Card Not Authenticated");
	}

	/* // For Seeing password of authentication
	for (int i = 0; i < 16; ++i) {
		Serial.write(readBackBlock[i]); // Output as Byte to ASCII
	}; */
	
	Serial.println();

	// Halt PICC
	mfrc522.PICC_HaltA();
	// Stop encryption on PCD
	mfrc522.PCD_StopCrypto1();

}

bool doorCheckError() {

	if (door1.checkSuccess() == true) {
		Serial.println(door1.errorReason());
	}
	
	return door1.checkSuccess();
}

void toggleDoor() {

	// Reset Auto Warnings
	beginArmingDoorClose = false;
	beginArmingDoorOpen = false;

	// Door Operation Toogle Lock or unlocked
	if (door1.isBoltUnlocked()) {
		door1.lockdoor();
	}
	else {
		door1.unlockdoor();
	}

	// Check if lock operation deteced errors
	doorCheckError();
}

void serialEvent() {

	String serialString = "";
	serialString.reserve(20);

	if (Serial.available()) {
		delay(10); //<----Should use interrupts-Lose time!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

				   // READS STRING
		char inChar = (char)Serial.read();


		// Binary DATA Input 8 bit "#01000000
		if (inChar == '#') {
			int i = 0;
			while (Serial.available()) {
				if (i < 15) {
					inChar = Serial.read();
					writeToBlock[i] = inChar;
					i++;
				}
			}
			globalByte[15] = '\0';
			Serial.print("15 + NULL Bytes Received: (ASCII): ");

			for (int i = 0; i < 16; ++i) {
				Serial.write(writeToBlock[i]); // Output as Byte to ASCII
			};
			Serial.println();

		}
		else if (inChar == '%') {

			int blockNumber = 0;

			blockNumber = Serial.parseInt();

			Serial.print("Read Block Number: ");
			Serial.println(blockNumber);

			cardReadBlockNumber = blockNumber;

		}
		else if (inChar == '^') {

			int blockNumber = 0;

			blockNumber = Serial.parseInt();

			Serial.print("Write Block Number: ");
			Serial.println(blockNumber);

			cardWriteBlockNumber = blockNumber;

		}
		else if (inChar == '$') {

			// Store Data in global array
			int i = 0;

			while (Serial.available()) {

				if (i < 3) {
					inChar = (char)Serial.read();
					globalChar[i] = inChar;
					i++;
				}
				else {
					// ignores rest of data
					inChar = (char)Serial.read();
				}
			}

			globalChar[i] = '\0'; // Adds in ending

			Serial.print(i);
			Serial.println(globalChar);

			// Convert Char to Int
			servoDoorAngle = (globalChar[0] - '0') * 100;
			servoDoorAngle += (globalChar[1] - '0') * 10;
			servoDoorAngle += (globalChar[2] - '0');

			servoDoor.write(servoDoorAngle);
			Serial.print("Servo Received servoDoorAngle[");
			Serial.print(servoDoorAngle);
			Serial.println("]");

			return; // Finish Loop
		}
		else {
			serialString += inChar;
			while (Serial.available()) {
				char inChar = (char)Serial.read();
				serialString += inChar;
			}
			Serial.print("String Received: ");
			Serial.println(serialString);

			//String decoder
			if (serialString == "write" || serialString == "w") {
				if (!writeToCardTrue) {
					writeToCardTrue = true;
					Serial.println("Card Writing On");
				}
				else {
					writeToCardTrue = false;
					Serial.println("Card Writing Off");
				}
			}
			else if (serialString == "dump" || serialString == "d") {
				if (!cardDumpTrue) {
					cardDumpTrue = true;
					Serial.println("Card Dump On");
				}
				else {
					cardDumpTrue = false;
					Serial.println("Card Dump Off");
				}
			}
			else if (serialString == "writeString" || serialString == "ws") {
				Serial.print("Write String is: ");
				for (int i = 0; i < 16; ++i) {
					Serial.write(writeToBlock[i]); // Output as Byte to ASCII
				};
				Serial.println();

			}
			else if (serialString == "writeNum" || serialString == "wn") {
				Serial.print("Write Block Number is: ");
				Serial.print(cardReadBlockNumber);

			}
			else if (serialString == "lock" || serialString == "l") {
				Serial.println("Locking Servo...");

			}
			else if (serialString == "open" || serialString == "o") {
				Serial.println("Opening Servo...");

			}
			else if (serialString == "relay" || serialString == "r") {

				Serial.print("Turning ");

				if (digitalRead(RELAY_PIN) == HIGH) {
					Serial.print("OFF");
				}
				else
				{
					Serial.print("ON");
				}

				Serial.print(" Relay Pin#");
				Serial.print(RELAY_PIN);
				Serial.println("...");

				digitalWrite(RELAY_PIN, !digitalRead(RELAY_PIN));
			}
			else if (serialString == "help" || serialString == "/h") {
				Serial.println("DOOR CONTROL 2016 V1 By Sean Zeng");
				Serial.println("Commands:");
				Serial.println("#RFIDSTRING $SERVOANGLE[3] %READBLOCKNUMBER ^WRITEBLOCKNUMBER");
				Serial.println("write, dump, writeString, writeNum, lock, open, relay");
			}
			else {
				Serial.print("ERROR COMMAND UNKNOWN: ");
				Serial.println(serialString);

			}

			serialString = "";
		}
	}
}