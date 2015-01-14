/*

TODO

*check proportional wheel driver function 
*check on wheel encoder overflow
*seperate motor incident parameters per motor
*make pump work! (write a seperate driver function for it? perhaps use timer?)
*get bumpers on interrupts (may require a uc with more interrupts ;-)

*/

/*********************************
*******    pinout v0.3   *********
**********************************

0 - 1		serial (rx, tx)
2 - 3 		direction wheels (right, left)
4 - 7 		motors PWM (elevator, pump, vacuum, brush) 
8 - 9 		wheel PWM (left, right)
10			flash light
11			LED 1
12			off switch
13 			FREE (PWM)
14 - 15		bluetooth (tx, rx)
16			LED 2 (serial 2 tx)
17			FREE (serial2 rx)
18 - 19		FREE (serial1)
20 - 21		I2C Wire library (IMU)
22			FREE
23			FREE
24 - 25		right encoder (channel 1, 2)
26 - 27 	left encoder (channel 1, 2)
28			FREE
-
30			FREE
31			LED 3
32			FREE
-
34			FREE
35			bumper left
36			FREE
37			bumper right
38			FREE
-
43			FREE
44 - 47 	direction motors (pump, elevator, brush, vacuum)
48			FREE
-
52			FREE
53			LED 4

A0 			current sensing left wheel
A1			LED 5
A2 			current sensing right wheel
A3 			battery sense
A4 - A7 	current sensing motors (elevator, pump, vacuum, brush)
A8			FREE
-
A11			FREE
DAC0		FREE
DAC1		FREE

*****************************************/

#include "Arduino.h"
#include "Wire.h"
#include "aJSON.h"
#include "MPU6050.h"
#include "log.h"
//#include "digitalWriteFast.h"  //could be used to improve performance

/*****************************************
 * Compile Flags
 *****************************************/

// #define ENCODERS_USED
#define RAMPING
#define IMU_INVERTED

/*****************************************
 * 
 *****************************************/

#define sgn(x) ((x < 0 )? (-1) : (1))

// message constants
#define HEADER 0xA5

// message type enum
#define SENSOR_DATA 0
#define DRIVE_COMMAND 1
#define MOTOR_COMMAND 2
#define CONTROL_COMMAND 3
#define DISCONNECT 4
#define SENSOR_REQUEST 5

// parameter type enum
#define INT_T 0
#define DOUBLE_T 1
#define STRING_T 2
#define BOOL_T 3

//// general ///////////////////
#define COMMAND_TIMEOUT 500    ///TODO: set this to 500ms or so for use with ROS
#define INCIDENT_TIMEOUT 5000
#define MAX_INCIDENT_COUNT 50

// battery pin
#define BATTERY_SENSE A3

// self destruct pin
#define SELF_DESTRUCT 13

// imu power switch pin
#define IMU_POWER_SWITCH 11

// indicator led pins
#define LED_1 11
#define LED_2 16
#define LED_3 31
#define LED_4 53
#define LED_5 A1

//// motors ////////////////////
// motor enum
#define ELEVATOR 0 //these numbers + 1 is how they are used in the cpp file
#define PUMP 1
#define VACUUM 2
#define BRUSH 3

// pwm pins
#define PWM_A 4
#define PWM_B 5
#define PWM_C 6
#define PWM_D 7
int PWM_MOTORS[4] = {PWM_A, PWM_B, PWM_C, PWM_D};

// current sense pins
#define CURRENT_SENSE_A A4
#define CURRENT_SENSE_B A5
#define CURRENT_SENSE_C A6
#define CURRENT_SENSE_D A7
int CURRENT_SENSE_MOTORS[4] = {CURRENT_SENSE_A, CURRENT_SENSE_B, CURRENT_SENSE_C, CURRENT_SENSE_D};

// direction pins
#define DIRECTION_B 44	// pump
#define DIRECTION_A 45  // elevator, + direction -> UP, - direction -> DOWN
#define DIRECTION_D 46 	// brush, + direction -> OK, - direction -> wrong
#define DIRECTION_C 47	// vacuum, both directions ok
int DIRECTION_MOTORS[4] = {DIRECTION_A, DIRECTION_B, DIRECTION_C, DIRECTION_D};

// define direction to be inverted 
// 		false 	-> 
//		true 	-> 
bool DIRECTION_INVERTED[4] = {false, false, true, false};

// current limit constant
#define CURRENT_LIMIT_A 1000
#define CURRENT_LIMIT_B 1000
#define CURRENT_LIMIT_C 1000
#define CURRENT_LIMIT_D 1000
int CURRENT_LIMIT_MOTORS[4] = {CURRENT_LIMIT_A, CURRENT_LIMIT_B, CURRENT_LIMIT_C, CURRENT_LIMIT_D};

// define motor to be inverted 
// 		false 	-> LOW = OFF, HIGH = ON
//		true 	-> LOW = ON, HIGH = OFF
bool MOTOR_INVERTED[4] = {false, false, false, false};

//// wheel motors //////////////
// pwm pins
#define PWM_LEFT 8
#define PWM_RIGHT 9

// direction pins
#define DIRECTION_RIGHT 2	// LOW -> FORWARD, HIGH -> BACKWARD
#define DIRECTION_LEFT 3	// LOW -> FORWARD, HIGH -> BACKWARD

// current sense pins
#define CURRENT_SENSE_LEFT A0
#define CURRENT_SENSE_RIGHT A2

// current limit constant
#define CURRENT_LIMIT_DRIVE 1000
#define MAX_INCIDENT_COUNT 50
#define BATTERY_SENSE A3

#define SELF_DESTRUCT 12

// max speed
#define MAX_SPEED 255

//// flash light ///////////////
// flash light pin
#define FLASH_LIGHT 10
// #define FLASH_ENABLED

//// bumper ////////////////////
// bumper pins
#define BUMPER_LEFT 35
#define BUMPER_RIGHT 37

//// accelero / gyro ///////////
// max number of elements in history
#define MAX_AG_HISTORY 5

// enum for accelero / gyro elements
#define AX 0
#define AY 1
#define AZ 2
#define GX 3
#define GY 4
#define GZ 5

// accelerometer range
#define ACCELEROMETER_RANGE 2.0 // +- 2g
// gyroscope range
#define GYROSCOPE_RANGE 250.0   // +- 250 deg/s

//// compass ///////////////////
// max cnumber of elements in history
// #define MAX_HEADING_HISTORY 5

//// encoder ///////////////////
// right encoder pins
#define RIGHT_ENCODER_A 24
#define RIGHT_ENCODER_B 25

// left encoder pins
#define LEFT_ENCODER_A 26
#define LEFT_ENCODER_B 27

////////////////////////////////
////////////////////////////////

// main
void setup();
void loop();

// communication
void receiveCommands();
void handleControlCommand(aJsonObject* json);
void handleDisconnect(aJsonObject* json);
void handleMotorCommand(aJsonObject* json);
void handleDriveCommand(aJsonObject* json);
void handleSensorRequest(aJsonObject* json);
void sendData();
void handleInput(int incoming);

// sensors
void senseMotor(int motor);
void senseLeftRight();
void readCompass();
void readAG();

// actuators
void drive();
void secdrive(int motor);
int capSpeed(int value);
void flashLight(int speed);
void setMotorSpeed(int motor_id);
void stop(int motor);

// helper functions
void pushFront(int val, int* valueList, int len);
void pushIncident();
void pushMotorIncident(int motor_id);
float formatAcceleroValue(int value);
float formatGyroValue(int value);
// float formatCompassValue(int value);
void resetSensors();
int getID(aJsonObject* json);
void decodeMotorCommand(aJsonObject* json, int* motor_id, int* speed);
void decodeDriveCommand(aJsonObject* json, int* left, int* right);

//TODO (pump)
//void reset();  //todo: pump functions
//void timerCB();
