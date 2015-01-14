#include "debug.h"
#include "robo40control.h"
#include "Encoder.h"

// JSON message is of the format:
// {"compass":{"heading":119.00000},"accelero":{"x":0.04712,"y":0.00049,"z":0.97757},"gyro":{"x":-0.39674,"y":-1.95318,"z":-1.65563}}

aJsonStream serial_stream(&Serial);

// compass and accelero variables
// int HMC6352Address = 0x42;
// int slaveAddress; // This is calculated in the setup() function
// byte headingData[2];
// int headingValue;
//int headingHistory[MAX_HEADING_HISTORY];
//long headingAvg = -1L;
//int headingMax = -1;
//int headingMin = 10000;
bool ag_connected = false;
int16_t agValue[6];
//int agHistory[6][MAX_AG_HISTORY];
//long agAvg[6];
//int agMax[6];
//int agMin[6];

//wheel motor / encoder variables
unsigned long incidents[MAX_INCIDENT_COUNT];
unsigned long lastcommand;
bool manualCommand = false;
int curLeftSpeed = 0;
int curRightSpeed = 0;
int desiredLeftSpeed = 0;
int desiredRightSpeed = 0;
//char lastDirectionLeft = 0;  //char to make sure operations on it are atomic
//char lastDirectionRight = 0;
int currentSenseLeft = 0;
int currentSenseRight = 0;
int reportCSLeft = 0;
int reportCSRight = 0;

//additional motor variables
int curSpeedMotor[4] = {0, 0, 0, 0};
int desiredSpeedMotor[4] = {0, 0, 0, 0};
int currentSenseMotor[4] = {0, 0, 0, 0};
int reportCSMotor[4] = {0, 0, 0, 0};
unsigned long motorIncidents[4][MAX_INCIDENT_COUNT];

// sent message number variable
int msgCounter;

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

#ifdef ENCODERS_USED
// Encoder
Encoder leftEncoder(LEFT_ENCODER_A, LEFT_ENCODER_B);
Encoder rightEncoder(RIGHT_ENCODER_A, RIGHT_ENCODER_B);
#endif

// --------------------------------------------------------------------
void setup() {

    //first setup pins, to minimize time that the vacuum pump is on
	for (int i = 0; i < 4; ++i) {
		pinMode(PWM_MOTORS[i], OUTPUT);
		stop(i);
	}

	// //setup pulse counter callbacks   //interrupts cause problem for drive function
	// // Quadrature encoders
	// // Left encoder
	// pinMode(c_LeftEncoderPinA, INPUT);      // sets pin A as input
	// pinMode(c_LeftEncoderPinB, INPUT);      // sets pin B as input
	// attachInterrupt(c_LeftEncoderInterrupt, HandleLeftMotorInterruptA, RISING);
	// // Right encoder
	// pinMode(c_RightEncoderPinA, INPUT);      // sets pin A as input
	// pinMode(c_RightEncoderPinB, INPUT);      // sets pin B as input
	// attachInterrupt(c_RightEncoderInterrupt, HandleRightMotorInterruptA, RISING);

	pinMode(DIRECTION_RIGHT, OUTPUT);
	pinMode(DIRECTION_LEFT, OUTPUT);

	pinMode(DIRECTION_A, OUTPUT);
	pinMode(DIRECTION_B, OUTPUT);
	pinMode(DIRECTION_C, OUTPUT);
	pinMode(DIRECTION_D, OUTPUT);

	pinMode(PWM_LEFT, OUTPUT);
	pinMode(PWM_RIGHT, OUTPUT);

	pinMode(CURRENT_SENSE_LEFT, INPUT);
	pinMode(CURRENT_SENSE_RIGHT, INPUT);

	pinMode(CURRENT_SENSE_A, INPUT);
	pinMode(CURRENT_SENSE_B, INPUT);
	pinMode(CURRENT_SENSE_C, INPUT);
	pinMode(CURRENT_SENSE_D, INPUT);

	pinMode(FLASH_LIGHT, OUTPUT);

    pinMode(BUMPER_LEFT, INPUT);
    pinMode(BUMPER_RIGHT, INPUT);

	pinMode(SELF_DESTRUCT, OUTPUT);

	pinMode(IMU_POWER_SWITCH, OUTPUT);

	//connect to computer, uses pin 0 and 1
	Serial.begin(115200);

// #ifdef DEBUG_SERIAL
// 	initLogging(&Serial);
// #endif

	LOGd(1, "startup...");

	///TODO: make pump work
    // timer for pump;
	//Timer1.initialize(1000);
	//Timer1.attachInterrupt(timerCB);

	// connection to compass, uses pins 20 and 21
	Wire.begin();

// #ifdef DEBUG_BT
// 	// connect to bluetooth, uses pin 14 and 15
// 	Serial3.begin(115200);
// 	initLogging(&Serial3);
// #endif

	// IMU / I2C is having problems with resets and gets stuck.
	// to avoid problems, turn IMU off
	digitalWrite(IMU_POWER_SWITCH, LOW);
	delay(500);

	// then turn it on again before initializing it
	digitalWrite(IMU_POWER_SWITCH, HIGH);
	delay(50);

	// setup accelero/gyro
	accelgyro.initialize(); // initialize device
	ag_connected = accelgyro.testConnection(); // verify connection
	LOGd(3, ag_connected ? "MPU6050 connection successful\r\n" : "MPU6050 connection failed");

	resetSensors(); // reset sensor history

	LOGd(1, "ready");
}

void loop() {
	// sendData();

	receiveCommands();							//talky talky

	if (!manualCommand) {
		drive();  									//wheels
	}
	for (int i = 1; i <= 4; i++) {
		secdrive(i);	//other motors
	}

	readAG();

}

//**********************************************************************************
// communication functions

void handleInput(int incoming) {
	LOGd(1, "incoming: %c (%d)", incoming, incoming);
	switch(incoming) {
		case 'o': sendData(); LOGd(1, ""); break;

		case 'q': manualCommand = false; lastcommand = millis(); desiredLeftSpeed = 0; desiredRightSpeed = 0; break;
		case 'e': manualCommand = false; curLeftSpeed = 0; curRightSpeed = 0; desiredLeftSpeed = 0; desiredRightSpeed = 0;	break;
		case 'w': manualCommand = false; lastcommand = millis(); desiredLeftSpeed = 200; desiredRightSpeed = 200; break;
		case 's': manualCommand = false; lastcommand = millis(); desiredLeftSpeed = -200; desiredRightSpeed = -200; break;
		case 'a': manualCommand = false; lastcommand = millis(); desiredLeftSpeed = -100; desiredRightSpeed = 100; break;
		case 'd': manualCommand = false; lastcommand = millis(); desiredLeftSpeed = 100; desiredRightSpeed = -100; break;

		case 'f': flashLight(200); break;
		case 'r': flashLight(0); break;

		case '1': manualCommand = true; digitalWrite(DIRECTION_LEFT, HIGH); break;
		case '2': manualCommand = true; digitalWrite(DIRECTION_LEFT, LOW); break;
		case '3': manualCommand = true; digitalWrite(DIRECTION_RIGHT, HIGH); break;
		case '4': manualCommand = true; digitalWrite(DIRECTION_RIGHT, LOW); break;  //pump; shouldnt work for now
		case '5': manualCommand = true; analogWrite(PWM_LEFT, 200); break;
		case '6': manualCommand = true; analogWrite(PWM_LEFT, 0); break;
		case '7': manualCommand = true; analogWrite(PWM_RIGHT, 200); break;
		case '8': manualCommand = true; analogWrite(PWM_RIGHT, 0); break;

		// case '`': LOGd(0, "self destruct"); 
		// 	digitalWrite(SELF_DESTRUCT, HIGH);
		// 	delay(500);
		// 	digitalWrite(SELF_DESTRUCT, LOW);
		// 	break;

		// case '1': desiredSpeedMotor[0] = 200; break;
		// case '2': desiredSpeedMotor[0] = 0; break;
		// case '3': desiredSpeedMotor[0] = -200; break;
		// case '4': desiredSpeedMotor[1] = 200; break;  //pump; shouldnt work for now
		// case '5': desiredSpeedMotor[1] = 0; break;
		// case '6': desiredSpeedMotor[2] = 255; break;
		// case '7': desiredSpeedMotor[2] = 0; break;
		// case '8': desiredSpeedMotor[3] = 255; break;
		// case '9': desiredSpeedMotor[3] = 0; break;

		case '0': stop(0); stop(1); stop(2); stop(3); break;

		// case 'l': log_level = (log_level+1) % 4; LOGd(0, "loglevel: %d", log_level); break;

		// default: LOGd(1, "incoming: %c (%d)", incoming, incoming); break;
	}
}

void receiveCommands() {

#ifdef DEBUG_SERIAL
	if (Serial.available()) {
		int incoming = Serial.read();
		handleInput(incoming);
	}
	return;
#endif

#ifdef DEBUG_BT
	if (Serial3.available()) {
		int incoming = Serial3.read();
		// LOGd(1, "serial3: %d", incoming);
		handleInput(incoming);
	}
#endif

	aJsonObject* item;
	if (serial_stream.available()) {
		item = aJson.parse(&serial_stream);

		if (item == NULL) {
			LOGd(0, "not a json object!");
			// while(Serial.available()) {
			// 	Serial3.print(Serial.read());
			// }
			// Serial3.println("");
			serial_stream.flush();
			return;
		}
	} else {
		return;
	}

	switch(getID(item)) {
		// switch(getType(item)) {
		case DRIVE_COMMAND:
			handleDriveCommand(item);
			break;
		case MOTOR_COMMAND:
			handleMotorCommand(item);
			break;
		case CONTROL_COMMAND:
			handleControlCommand(item);
			break;
		case DISCONNECT:
			handleDisconnect(item);
			break;
		case SENSOR_REQUEST:
			handleSensorRequest(item);
			break;
		default:
			break;
	}
	aJson.deleteItem(item);

}

void handleControlCommand(aJsonObject* json) { ///TODO: remove?
// is not necessary for now
}

void handleDisconnect(aJsonObject* json) {
// is sent when the phone disconnects from the robot, best to turn off all motors here

	//bluntly put wheels to zero and wait for drive function to be called again
	curLeftSpeed = 0;
	curRightSpeed = 0;
	desiredLeftSpeed = 0;
	desiredRightSpeed = 0;

    //also stop the other motors
    for (int i = 0; i < 4; ++i) {
		stop(i);
	}

    flashLight(0); //and the flashlight of course :-)
}

void handleSensorRequest(aJsonObject* json) {
	LOGd(3, "handleSensorRequest");
	sendData();
}

void handleMotorCommand(aJsonObject* json) {
	int id = -1, value = -1;
	decodeMotorCommand(json, &id, &value); //  motor id, speed value between -255 and 255
	LOGd(3, "handleMotorCommand (%d, %d)", id, value);
	desiredSpeedMotor[id] = capSpeed(value);
}

void handleDriveCommand(aJsonObject* json) {
	int leftSpeed = 0, rightSpeed = 0;
	decodeDriveCommand(json, &leftSpeed, &rightSpeed);
	LOGd(3, "hanldeDriveCommand (%d, %d)", leftSpeed, rightSpeed);
	lastcommand = millis();
	desiredRightSpeed = capSpeed(rightSpeed);
	desiredLeftSpeed = capSpeed(leftSpeed);
}

// JSON message is of the format:
// {"compass":{"heading":119.00000},"accelero":{"x":0.04712,"y":0.00049,"z":0.97757},"gyro":{"x":-0.39674,"y":-1.95318,"z":-1.65563}}

void sendData() {

	aJsonObject *json, *header, *data, *group, *sub, *item;

	json = aJson.createObject();

	// header = aJson.createObject();
	// aJson.addNumberToObject(header, "id", HEADER);
	// aJson.addNumberToObject(header, "timestamp", msgCounter++);
	// aJson.addNumberToObject(header, "type", SENSOR_DATA);
	// aJson.addItemToObject(json, "header", header);
	aJson.addNumberToObject(json, "id", SENSOR_DATA);

	data = aJson.createObject();

	// BUMPER
	group = aJson.createObject();
	aJson.addNumberToObject(group, "left", digitalRead(BUMPER_LEFT));
	aJson.addNumberToObject(group, "right", digitalRead(BUMPER_RIGHT));
	aJson.addItemToObject(data, "bumper", group);

	// COMPASS
	// group = aJson.createObject();
	// aJson.addNumberToObject(group, "heading", formatCompassValue(headingValue));
	// aJson.addItemToObject(data, "compass", group);

	// ACCELERO
	group = aJson.createObject();
	aJson.addNumberToObject(group, "x", formatAcceleroValue(agValue[AX]));
	aJson.addNumberToObject(group, "y", formatAcceleroValue(agValue[AY]));
	aJson.addNumberToObject(group, "z", formatAcceleroValue(agValue[AZ]));
	aJson.addItemToObject(data, "accelero", group);

	// GYRO
	group = aJson.createObject();
	aJson.addNumberToObject(group, "x", formatGyroValue(agValue[GX]));
	aJson.addNumberToObject(group, "y", formatGyroValue(agValue[GY]));
	aJson.addNumberToObject(group, "z", formatGyroValue(agValue[GZ]));
	aJson.addItemToObject(data, "gyro", group);

#ifdef ENCODERS_USED
	// ENCODER
	group = aJson.createObject();
	aJson.addNumberToObject(group, "left", (int)leftEncoder.read());
	aJson.addNumberToObject(group, "right", (int)rightEncoder.read());
	aJson.addItemToObject(data, "odom", group);
#endif

	// WHEELS
	group = aJson.createObject();

	// .. LEFT
	sub = aJson.createObject();
	// item = aJson.createObject();
	// aJson.addNumberToObject(item, "present", currentSenseLeft);
	// aJson.addNumberToObject(item, "peak", reportCSLeft);
	// aJson.addItemToObject(sub, "current", item);
	aJson.addNumberToObject(sub, "speed", curLeftSpeed);
	aJson.addItemToObject(group, "left", sub);

	// .. RIGHT
	sub = aJson.createObject();
	// item = aJson.createObject();
	// aJson.addNumberToObject(item, "present", currentSenseRight);
	// aJson.addNumberToObject(item, "peak", reportCSRight);
	// aJson.addItemToObject(sub, "current", item);
	aJson.addNumberToObject(sub, "speed", curRightSpeed);
	aJson.addItemToObject(group, "right", sub);

	aJson.addItemToObject(data, "wheels", group);

	// // MOTORS
	// group = aJson.createObject();

	// // .. ELEVATOR
	// sub = aJson.createObject();
	// item = aJson.createObject();
	// aJson.addNumberToObject(item, "present", currentSenseMotor[ELEVATOR]);
	// aJson.addNumberToObject(item, "peak", reportCSMotor[ELEVATOR]);
	// aJson.addItemToObject(sub, "current", item);
	// aJson.addNumberToObject(sub, "speed", curSpeedMotor[ELEVATOR]);
	// aJson.addItemToObject(group, "elevator", sub);

	// // .. PUMP
	// sub = aJson.createObject();
	// item = aJson.createObject();
	// aJson.addNumberToObject(item, "present", currentSenseMotor[PUMP]);
	// aJson.addNumberToObject(item, "peak", reportCSMotor[PUMP]);
	// aJson.addItemToObject(sub, "current", item);
	// aJson.addNumberToObject(sub, "speed", curSpeedMotor[PUMP]);
	// aJson.addItemToObject(group, "pump", sub);

	// // .. VACUUM
	// sub = aJson.createObject();
	// item = aJson.createObject();
	// aJson.addNumberToObject(item, "present", currentSenseMotor[VACUUM]);
	// aJson.addNumberToObject(item, "peak", reportCSMotor[VACUUM]);
	// aJson.addItemToObject(sub, "current", item);
	// aJson.addNumberToObject(sub, "speed", curSpeedMotor[VACUUM]);
	// aJson.addItemToObject(group, "vacuum", sub);

	// // .. BRUSH
	// sub = aJson.createObject();
	// item = aJson.createObject();
	// aJson.addNumberToObject(item, "present", currentSenseMotor[BRUSH]);
	// aJson.addNumberToObject(item, "peak", reportCSMotor[BRUSH]);
	// aJson.addItemToObject(sub, "current", item);
	// aJson.addNumberToObject(sub, "speed", curSpeedMotor[BRUSH]);
	// aJson.addItemToObject(group, "brush", sub);

	// aJson.addItemToObject(data, "motors", group);

	aJson.addItemToObject(json, "data", data);

	aJson.print(json, &serial_stream);
	Serial.println("");
	aJson.deleteItem(json);
}

//*******************************************************************************
// sensor functions

void senseLeftRight() {
	currentSenseLeft = analogRead(CURRENT_SENSE_LEFT);
	currentSenseRight = analogRead(CURRENT_SENSE_RIGHT);
	reportCSLeft = max(currentSenseLeft, reportCSLeft); // track maximum current sensed
	reportCSRight = max(currentSenseRight, reportCSRight);
}

void senseMotor(int motor) {
	currentSenseMotor[motor] = analogRead(CURRENT_SENSE_MOTORS[motor]);
	reportCSMotor[motor] = max(reportCSMotor[motor], currentSenseMotor[motor]);
}

// void readCompass() {
// // Send a "A" command to the HMC6352
// // This requests the current heading data
// 	Wire.beginTransmission(slaveAddress);
// 	Wire.write("A");              // The "Get Data" command
// 	Wire.endTransmission();
// 	delay(2);                   // The HMC6352 needs at least a 70us (microsecond) delay after this command.
// 	// Read the 2 heading bytes, MSB first. The resulting 16bit word is the compass heading in 10th's of a degree
// 	// For example: a heading of 1345 would be 134.5 degrees
// 	Wire.requestFrom(slaveAddress, 2);        // Request the 2 byte heading (MSB comes first)
// 	int i = 0;
// 	while(Wire.available() && i < 2)
// 	{
// 		headingData[i] = Wire.read();
// 		i++;
// 	}
// 	headingValue = headingData[0]*256 + headingData[1];  // Put the MSB and LSB together

// //	pushFront(headingValue, headingHistory, MAX_HEADING_HISTORY);
// //
// //	//calculate new min / max values
// //	if (headingValue > headingMax) {
// //		headingMax = headingValue;
// //	}
// //
// //	if (headingValue < headingMin) {
// //		headingMin = headingValue;
// //	}
// //
// //	//calculate new rolling average
// //	long nHeadingAvg = 0L;
// //	int elements = 0;
// //	for (int i = 0; i < MAX_HEADING_HISTORY; i++)
// //	{
// //		if (headingHistory[i] == -1.0) continue;
// //		elements++;
// //		nHeadingAvg += headingHistory[i];
// //	}
// //
// //	headingAvg = (long) nHeadingAvg / elements;

// }

void readAG() {

	if (!ag_connected) return;

	// read raw accel/gyro measurements from device
	accelgyro.getMotion6(&agValue[AX], &agValue[AY], &agValue[AZ], &agValue[GX], &agValue[GY], &agValue[GZ]);

//	for (int j = 0; j < 6; ++j)
//	{
//			// agValue[j] = agValue[j] / (32767 / 2) * 9.81;
//
//		pushFront(agValue[j], agHistory[j], MAX_AG_HISTORY);
//
//			//calculate the new min / max values
//		if (agValue[j] > agMax[j]) {
//			agMax[j] = agValue[j];
//		}
//
//		if (agValue[j] < agMin[j]) {
//			agMin[j] = agValue[j];
//		}
//
//			//calculate the new rolling average
//		int elements = 0;
//		long average = 0L;
//		for (int i = 0; i < MAX_AG_HISTORY; i++)
//		{
//			if (agHistory[j][i] == -1.0) continue;
//			elements++;
//			average += agHistory[j][i];
//		}
//		agAvg[j] = (long) average /  elements;
//	}
}

// Interrupt service routines for the left motor's quadrature encoder
// void HandleLeftMotorInterruptA() {
// 	//for managing with 1 channel encoder (really doesnt work at all)
// 	//#ifdef LeftEncoderIsReversed
// 	// _LeftEncoderTicks -= lastDirectionLeft;
// 	// #else
// 	// _LeftEncoderTicks += lastDirectionLeft;
// 	//#endif
// 	//for working 2channel encoder
// 	// Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
// 	_LeftEncoderBSet = analogRead(c_LeftEncoderPinB) < 750 ? false : true; // read the input pin
// 	// and adjust counter + if A leads B
// 	#ifdef LeftEncoderIsReversed
// 		_LeftEncoderTicks -= _LeftEncoderBSet ? -1 : +1;
// 	#else
// 		_LeftEncoderTicks += _LeftEncoderBSet ? -1 : +1;
// 	#endif
// }
// Interrupt service routines for the right motor's quadrature encoder
// void HandleRightMotorInterruptA() {
// 	//for managing with 1 channel encoder (really doesnt work at all)
// 	//#ifdef RightEncoderIsReversed
// 	// _RightEncoderTicks -= lastDirectionRight;
// 	//#else
// 	// _RightEncoderTicks += lastDirectionRight;
// 	//#endif
// 	//for working 2channel encoder
// 	// Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
// 	_RightEncoderBSet = analogRead(c_RightEncoderPinB) < 750 ? false : true; // read the input pin //digitalReadFast
// 	// and adjust counter + if A leads B
// 	#ifdef RightEncoderIsReversed
// 		_RightEncoderTicks -= _RightEncoderBSet ? -1 : +1;
// 	#else
// 		_RightEncoderTicks += _RightEncoderBSet ? -1 : +1;
// 	#endif
// }

//***********************************************************************************
//   actuator functions

unsigned long lastIncidentLogTime = 0;
unsigned long lastBumperLogTime = 0;

// driver function for wheels
void drive() {
	//first check current sensors and tune down desired speed if current is too high
	senseLeftRight();
	if ( (currentSenseLeft > CURRENT_LIMIT_DRIVE) || (currentSenseRight > CURRENT_LIMIT_DRIVE) )
	{
		//tune down desiredspeed of both wheels by a percentage
		desiredRightSpeed = desiredRightSpeed * 0.8;
		desiredLeftSpeed = desiredLeftSpeed * 0.8;

		pushIncident(); //store incident
		LOGd(2, "Too much current in one of the wheels: %d, %d", currentSenseLeft, currentSenseRight);
	}

	//secondly, check if desiredspeed received too long ago, or too many incidents recently, and if so, refuse to move
	unsigned long time = millis();
	unsigned long incidentDifference = time - incidents[MAX_INCIDENT_COUNT - 1];
	unsigned long commandDifference = time - lastcommand;
		//differences should underflow when time overflowed

// ignore command timeouts when debugging
#ifndef DEBUG
	//time should be bigger than INCIDENTTIMEOUT b/c incidents are initialized to 0
	//also note that outliers are ignored implicitly here: one incident doesnt do much at all
	if ((commandDifference > COMMAND_TIMEOUT) || ((time > INCIDENT_TIMEOUT) && (incidentDifference < INCIDENT_TIMEOUT)))
	{
		//bluntly put everything to zero
        curLeftSpeed = 0;
		curRightSpeed = 0;
		desiredLeftSpeed = 0;
		desiredRightSpeed = 0;

		if (time - lastIncidentLogTime > 500) {
			LOGd(1, "Wheel speed set to zero due to problem: no commands or too many incidents!");
			LOGd(3, "senseLeft: %d, senseRight: %d", currentSenseLeft, currentSenseRight);
			LOGd(3, "last command received: %d, now: %d", lastcommand, time);
			lastIncidentLogTime = time;
		}
	}
#endif

	// So far so good, move proportionally closer to desired speed
#ifdef RAMPING
	int diffRightSpeed = desiredRightSpeed - curRightSpeed;
    curRightSpeed += sgn(diffRightSpeed) * min(abs(diffRightSpeed), 20);
#else
	// [19.11.14] no more ramping necessary with new hardware
	curRightSpeed = desiredRightSpeed;
#endif

#ifdef RAMPING
	int diffLeftSpeed = desiredLeftSpeed - curLeftSpeed;
    curLeftSpeed += sgn(diffLeftSpeed) * min(abs(diffLeftSpeed), 20);
#else
	// [19.11.14] no more ramping necessary with new hardware
	curLeftSpeed = desiredLeftSpeed;
#endif

#ifndef DEBUG
	//finally check bumpers to prevent movement forward
	// int bumper_ok = digitalRead(BUMPER_LEFT) | digitalRead(BUMPER_RIGHT);
	// int bumper_ok = 1;

	// LOGd(1, "bumper: %d (l=%d, r=%d)", bumper_ok, digitalRead(BUMPER_LEFT), digitalRead(BUMPER_RIGHT));
	if ((digitalRead(BUMPER_LEFT) == 0) || (digitalRead(BUMPER_RIGHT) == 0)) { //bumper pressed
	// if (!bumper_ok) {
		//blunty overwrite any intentions for going forward, then proceed as usual
		if (curRightSpeed > 0) curRightSpeed = 0;
		if (curLeftSpeed > 0) curLeftSpeed = 0;

		if (desiredRightSpeed > 0) desiredRightSpeed = 0;
		if (desiredLeftSpeed > 0) desiredLeftSpeed = 0;

		if (time - lastBumperLogTime > 500) {
			LOGd(2,"Bumper pressed, only allowing backwards movement!");
			lastBumperLogTime = time;
		}
	}
#endif

	if (curLeftSpeed > 0) {
		digitalWrite(DIRECTION_LEFT, LOW);
		//lastDirectionLeft = 1; //to let the encoder know which way we're going
	} else if (curLeftSpeed < 0)  {
		digitalWrite(DIRECTION_LEFT, HIGH);
	}

	if (curRightSpeed > 0) {
		digitalWrite(DIRECTION_RIGHT, LOW);
		//lastDirectionRight = 1; //to let the encoder know which way we're going
	} else if (curRightSpeed < 0)  {
		digitalWrite(DIRECTION_RIGHT, HIGH);
	}

	if ((curRightSpeed | curLeftSpeed) != 0) {
		flashLight(255);
	} else {
		flashLight(0);
	}

	// [19.11.14] temporarily added to partially solve direction switch problem
	// delay(5);

	analogWrite(PWM_LEFT, abs(curLeftSpeed));   //PWM Speed Control
	analogWrite(PWM_RIGHT, abs(curRightSpeed));   //PWM Speed Control

	return;

}

//driver function for lift, brush and vacuum pump
void secdrive(int motor) {
	//LOGd(2, "secdrive(%d, %d)",	motor);

	if ((motor < 1) || (motor > 4)) return; //other motors dont exist, so return immediately
	if (motor == 2) return; //the water pump probably needs a seperate driver function due to its AC nature

	int motor_id = motor - 1;

	//first check current sensor and tune down desired speed if current is too high
	senseMotor(motor_id);
	if (currentSenseMotor[motor_id] > CURRENT_LIMIT_MOTORS[motor_id])
	{
		desiredSpeedMotor[motor_id] = desiredSpeedMotor[motor_id] * 0.8;
		pushMotorIncident(motor_id);
	}

	//secondly, check if too many incidents recently, and if so, refuse to move
	unsigned long time = millis();
	unsigned long incidentDifference = time - motorIncidents[motor_id][MAX_INCIDENT_COUNT - 1];
		//differences should underflow when time overflowed

	//time should be bigger than INCIDENTTIMEOUT b/c incidents are initialized to 0
	//also note that outliers are ignored implicitly here: one incident doesnt do much at all
	if ((time > INCIDENT_TIMEOUT) && (incidentDifference < INCIDENT_TIMEOUT))
	{
		//bluntly put everything to zero
		desiredSpeedMotor[motor_id] = 0;
		curSpeedMotor[motor_id] = 0;
	}

	int diffSpeed = desiredSpeedMotor[motor_id] - curSpeedMotor[motor_id];
	curSpeedMotor[motor_id] += sgn(diffSpeed) * min(abs(diffSpeed), 20);

/*
	//ok, now move closer to desired speed
	if (desiredSpeedMotor[motor_id] < curSpeedMotor[motor_id]) {
		curSpeedMotor[motor_id]--;
	} else if (desiredSpeedMotor[motor_id] > curSpeedMotor[motor_id]) {
		curSpeedMotor[motor_id]++;
	}
*/

	setMotorSpeed(motor_id);

	//LOGd(3, "curSpeed: %d, sense: %d, report: %d",
	//	curSpeedMotor[motor_id], currentSenseMotor[motor_id], reportCSMotor[motor_id]);

}

void setMotorSpeed(int motor_id) {

	if (((curSpeedMotor[motor_id] > 0) && (!DIRECTION_INVERTED[motor_id])) ||
		((curSpeedMotor[motor_id] < 0) && DIRECTION_INVERTED[motor_id]) ) {   //set the direction of the motor
		digitalWrite(DIRECTION_MOTORS[motor_id], HIGH);
	} else {
		digitalWrite(DIRECTION_MOTORS[motor_id], LOW);
	}

	if (MOTOR_INVERTED[motor_id]) {      //set the PWM signal, taking into account inversion circuits
		analogWrite(PWM_MOTORS[motor_id], 255 - abs(curSpeedMotor[motor_id]));
	} else {
		analogWrite(PWM_MOTORS[motor_id], abs(curSpeedMotor[motor_id]));
	}
}

void stop(int motor_id) {

	if ((motor_id < 1) || (motor_id > 4)) return;
	curSpeedMotor[motor_id] = 0;
	desiredSpeedMotor[motor_id] = 0;
	setMotorSpeed(motor_id);
}

int capSpeed(int value) {
    return max(min(value,MAX_SPEED),-MAX_SPEED);
}

void flashLight(int speed) {
#ifdef FLASH_ENABLED
	analogWrite(FLASH_LIGHT, speed);
#endif
}


//******************************************************************
// helper functions

void pushFront(int val, int* valueList, int len) {
	for (int i = len-2; i >= 0; i--)
	{
		valueList[i+1] = valueList[i];
	}
	valueList[0] = val;
}

void pushIncident() {
	for (int i = MAX_INCIDENT_COUNT - 2; i >= 0; i-- )
	{
		incidents[i + 1] = incidents[i];
	}
	incidents[0] = millis();
}

void pushMotorIncident(int motor_id) {
	for (int i = MAX_INCIDENT_COUNT - 2; i >= 0; i-- )
	{
		motorIncidents[motor_id][i + 1] = motorIncidents[motor_id][i];
	}
	motorIncidents[motor_id][0] = millis();

}

float formatAcceleroValue(int value) {
	return float(value / 32767.0 * ACCELEROMETER_RANGE);
}

float formatGyroValue(int value) {
	float result = float(value / 32767.0 * GYROSCOPE_RANGE);

#ifdef IMU_INVERTED
	result *= -1;
#endif
	
	return result;
}

float formatCompassValue(int value) {
	return float(value / 10.0);
}

void resetSensors() {

//	//compass
//	for (int i = 0; i < MAX_HEADING_HISTORY; i++) {
//		headingHistory[i] = -1;
//	}
//
//	headingAvg = -1L;
//	headingMax = -1000000;
//	headingMin = 1000000;

//	//AG
//	for (int i = 0; i < 6; ++i)
//	{
//		agValue[i] = 0;
//		for (int j = 0; j < MAX_AG_HISTORY; ++j)
//		{
//			agHistory[i][j] = -1;
//		}
//		agAvg[i] = -1L;
//		agMax[i] = -1000000;
//		agMin[i] = 1000000;
//	}

	//current incidents
	for (int i = 0; i < MAX_INCIDENT_COUNT; i++)
	{
		incidents[i] = 0;
		for (int j = 0; j < 4; j++)
		{
			motorIncidents[j][i] = 0;
		}
	}

}

int getID(aJsonObject* json) {
	aJsonObject* id;
	id = aJson.getObjectItem(json, "id");
	if (id == NULL) {
		LOGd(1, "wrong json message");
		return -1;
	}
	return id->valueint;
}

void decodeMotorCommand(aJsonObject* json, int* motor_id, int* speed) {
	aJsonObject* data = aJson.getObjectItem(json, "data");

	aJsonObject* motor_id_j = aJson.getArrayItem(data, 0);
	*motor_id = motor_id_j->valueint;

	aJsonObject* speed_j = aJson.getArrayItem(data, 1);
	*speed = speed_j->valueint;
}

void decodeDriveCommand(aJsonObject* json, int* left, int* right) {
	aJsonObject* data = aJson.getObjectItem(json, "data");

	aJsonObject* left_j = aJson.getArrayItem(data, 0);
	*left = left_j->valueint;

	aJsonObject* right_j = aJson.getArrayItem(data, 1);
	*right = right_j->valueint;
}

// void decodeDriveCommand(aJsonObject* json, int* left, int* right) {
// 	aJsonObject* data = aJson.getObjectItem(json, "data");

// 	aJsonObject* left_j = aJson.getObjectItem(data, "left");
// 	*left = left_j->valueint;

// 	aJsonObject* right_j = aJson.getObjectItem(data, "right");
// 	*right = right_j->valueint;
// }

//****************************************************************
//  TODO

/* TODO: make pump work
int oncount = 5;
int interval = 100;

int count = 0;

boolean on = false;
boolean run = false;
boolean lastOn = false;
boolean log_sense = true;

void reset() {
	oncount = 5;
	interval = 100;
	count = 0;
	run = false;
	on = false;
	lastOn = false;
	log_sense = true;
}

void timerCB() {
	if (run) {
		count++;
		if (count <= oncount) {
			on = true;
		} else {
			on = false;
			if (count == interval) {
				count = 0;
				log_sense = true;
			}
		}

		if (lastOn != on) {
			LOGd(1, "on %d at %d", on, millis());
			digitalWrite(PWM_MOTORS[1], on ? HIGH : LOW);
		}
		lastOn = on;

		if (on && log_sense) {
			senseMotor(1);

			LOGd(3, "sense %d",
				currentSenseMotor[1]);

			log_sense = false;
		} else {
			log_sense = true;
		}
	} else {
		digitalWrite(PWM_MOTORS[1], LOW);
	}
}
*/
