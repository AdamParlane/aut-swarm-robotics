/*
* robot_setup.h
*
* Author : Adam Parlane, Matthew Witt
* Created: 6/7/2017
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Contains the misc defines that havnt been modularised as of yet (6/7)
* Also has all the headers so it can just be included in every header giving access to everything.
* There are compiler directives that will compile defines specific to each version of the robot.
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
*
* Functions:
* void masterClockInit(void)
* uint8_t waitForFlag(const volatile uint32_t *regAddr, uint32_t regMask, uint16_t timeOutMs)
* int32_t capToRangeInt(int32_t valueToCap, int32_t minimumVal, int32_t maximumVal)
* uint32_t capToRangeUint(uint32_t valueToCap, uint32_t minimumVal, uint32_t maximumVal)
* float capToRangeFlt(float valueToCap, float minimumVal, float maximumVal)
*
*/

#ifndef ROBOTDEFINES_H_
#define ROBOTDEFINES_H_

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "Interfaces/spi.h"		//Fixes SPI issue
#include "sam.h"				//Micro controller specific defines
#include <stdint.h>				//Gives standard integer type definitions (ie uint8_t)
#include <stdbool.h>			//Gives boolean variable types
#include "Interfaces/xbee_driver.h"//Gives access to MessageInfo structure definition

//////////////[Enumerations]////////////////////////////////////////////////////////////////////////
//The following enumerations represent states in each state machine in the system
typedef enum MainStates
//main() function states
{
	M_TEST,
	M_TEST_ALL,
	M_MANUAL,
	M_FORMATION,
	M_DOCKING,
	M_OBSTACLE_AVOIDANCE,
	M_IDLE, 
	M_CHARGING,
	M_LINE_FOLLOW,
	M_LIGHT_FOLLOW
} MainStates;

typedef enum DockingStates
//dfDockRobot() function states
{
	DS_FINISHED,
	DS_START,
	DS_FACE_BRIGHTEST,
	DS_MOVE_FORWARD,
	DS_RESCAN_BRIGHTEST,
	DS_FOLLOW_LINE,
	DS_CHRG_CONNECT,
	DS_CHRG_NOT_FOUND
} DockingStates;

typedef enum FollowLineStates
//dfFollowLine() function states
{
	FLS_START,
	FLS_FIRST_CONTACT,
	FLS_ALIGN,
	FLS_FOLLOW,
	FLS_GIVE_UP,
	FLS_FINISH
} FollowLineStates;

typedef enum ScanBrightestStates
//mfScanBrightestLightSource() function states
{
	SBS_FUNCTION_INIT,
	SBS_GOTO_START_POSITION,
	SBS_SWEEP,
	SBS_END
} ScanBrightestStates;

typedef enum ChargeCycleStates
//pfChargeCycleHandler() function states
{
	CCS_FINISHED,
	CCS_CHECK_POWER,
	CCS_CHARGING,
	CCS_RECONNECT,
	CCS_FAULT,
	CCS_DISMOUNT,
	CCS_TURN_AWAY,
	CCS_STOP_POLLING
} ChargeCycleStates;

typedef enum MTHByDistanceStates
//mfMoveToHeadingByDistance() function states
{
	MHD_START,
	MHD_MOVING,
	MHD_STOP
} MTHByDistanceStates;

////////////////[Type Definitions]//////////////////////////////////////////////////////////////////
//Stores optical sensor raw and derived data
typedef struct OpticalSensor
{
	int dx;				//Rate of change from optical sensor (X axis is left to right)
	int dy;				//Rate of change from optical sensor (Y axis is fwd/bckwd)
	int x;				//Count sum on x axis
	int y;				//Count sum on y axis
	char pollEnabled;	//Enable polling the optical sensor
	char overflowFlag;	//1 if data has overflowed on optical sensor
	uint8_t surfaceQuality;	//A value signifying quality of the surface (242 = max quality)
} OpticalSensor;

//Stores IMU sensor raw and converted data
typedef struct IMUSensor
{
	long qw;			//W component of the quaternion complex number returned by DMP
	long qx;			//X component of the quaternion complex number returned by DMP
	long qy;			//Y component of the quaternion complex number returned by DMP
	long qz;			//Z component of the quaternion complex number returned by DMP
	float accelX;		//Delta X Acceleration in ms^2
	float accelY;		//Delta Y Acceleration in ms^2
	float accelZ;		//Delta Z Acceleration in ms^2
	float gyroX;		//Delta pitch in deg/s
	float gyroY;		//Delta roll in	deg/s
	float gyroZ;		//Delta yaw in deg/s (Delta heading)
	float pitch;		//Absolute pitch from DMP (degrees)
	float roll;			//Absolute roll from DMP (degrees)
	float yaw;			//Absolute yaw (heading) from DMP (degrees)
	char dmpEnabled;	//A flag that states whether or not the DMP is enabled
	char pollEnabled;	//Enable polling the IMU
	char gyroCalEnabled;	//Enable gyro calibration on startup.
} IMUSensor;

//structure to store all the robot side navigation / positioning data
//this will be written to by getMouseXY, nfGetEulerAngles, and another navigation function which
//combines them. The structure will store the relevant info from both key sensors and fuse them in
//an additional function (84bytes i think)
typedef struct Position
{
	OpticalSensor Optical;		//Optical sensor raw data
	IMUSensor IMU;				//IMU raw and converted data
	int32_t x;					//Absolute X position in arena (mm)
	int32_t y;					//Absolute Y position in arena (mm)
	int32_t dx;					//delta x in mm
	int32_t dy;					//delta y in mm
	float speed;				//Speed in mm per second
	float heading;				//Absolute direction of travel (deg)
	float relHeading;			//Relative heading of travel (to front of robot)
	float facing;				//Absolute direction robot is facing (deg)
	signed int targetHeading;	//For obstacle avoidance, desired heading before an obstacle is 
								//detected
	char targetSpeed;			//For obstacle avoidance, desired speed
	unsigned long timeStamp;	//Time at which last IMU reading took place (ms). Can be used as a
								//time marker for all Nav data, as it all get polled at the same
								//time as the IMU
	unsigned short deltaTime;	//Time between last IMU reading and IMU previous reading
	float facingOffset;			//Used to offset facing value (when corrected by PC)
} Position;

//Stores information about the battery and charging
typedef struct BatteryChargeData
{
	uint16_t batteryVoltage;			//Battery voltage in mV
	uint8_t batteryTemp;				//Battery temperature in degrees
	uint16_t batteryMaxVoltage;			//Fully charged voltage of battery (will calibrate onthefly)
	uint16_t batteryDockingVoltage;		//Voltage below which the robot should seek dock
	uint16_t batteryMinVoltage;			//Voltage at which robot is considered completely dead
	uint8_t fcChipStatus;				//Status or fault code reported by Charge chip
	uint8_t fcChipFaultFlag;			//Fault detected by charge chip, see Status for code
	uint8_t pollBatteryEnabled;			//Enable battery voltage polling
	uint16_t pollBatteryInterval;		//Interval in ms to poll battery voltage/temp
	uint8_t pollChargingStateEnabled;	//Enable charge chip status polling
	uint16_t pollChargingStateInterval;	//Interval in ms to poll charge chip
	uint8_t chargeWatchDogEnabled;		//Enable the FC chip watchdog system (for when charging)
	uint16_t chargeWatchDogInterval;	//Watchdog routine interval (ms)
} BatteryChargeData;

//Stores colour sensor data, both raw and converted, for a single colour sensor
typedef struct ColourSensorData
{
	unsigned short red;
	unsigned short green;
	unsigned short blue;
	unsigned short white;
	unsigned short hue;
	unsigned short saturation;
	unsigned short value;
} ColourSensorData;

//Will store states of the line sensors. This is necessary
//because there is a gray area when the sensor is half on and half off the line, so by establishing
//hysteresis and only changing the stored states when an upper and lower threshold is crossed,
//jitter should be reduced.
struct LineSensorArray
{
	uint8_t outerLeft;
	uint8_t innerLeft;
	uint8_t innerRight;
	uint8_t outerRight;
};

typedef struct CommunicationData
{
	uint8_t pollEnabled;				//Whether or not to poll for new messages in main()
	uint16_t pollInterval;				//Interval at which to poll at (ms)
	struct MessageInfo messageData;		//Next message data
	uint16_t testModeStreamInterval;	//Interval between sending test data packets (ms)
} CommunicationData;

//Structure that will store all system flags for global use
typedef struct SystemFlags
{
	char xbeeNewData;	//New data from Xbee interface
	char imuCheckFifo;	//IMU ext interrupt has been triggered
	char obaMoving;		//Robot is in motion
	char obaEnabled;	//Obstacle avoidance enabled
	char tfStream;		//Test function stream time flag
} SystemFlags;

//Structure that will store the state of every state machine in the system
typedef struct SystemStates
{
	//Main function state machine state
	MainStates mainf;
	MainStates mainfPrev;
	
	//dfDockRobot() states
	DockingStates docking;
	
	//dfFollowLine() states
	FollowLineStates followLine;
	
	//dfScanBrightestLightSource() states
	ScanBrightestStates scanBrightest;
	
	//pfChargeCycleHandler() states
	ChargeCycleStates chargeCycle;
	
	//mfMoveToHeadingByDistance() states
	MTHByDistanceStates moveHeadingDistance;
} SystemStates;

//Sensors sub-structure
typedef struct SensorData
{
	LineSensorArray line;
	ColourSensorData colourLeft;
	ColourSensorData colourRight;
} SensorData;

//Structure to combine all system globals
typedef struct RobotGlobalStructure
{
	SystemStates states;			//System states
	SystemFlags flags;				//System global flags
	SensorData sensors;				//Sensor data
	CommunicationData comms;		//Communication system control and data
	Position pos;					//Position information
	BatteryChargeData power;		//Battery/Charging info and control
	uint32_t timeStamp;				//System timestamp (millisecs since power on)
} RobotGlobalStructure;

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////

//////////////[Global variables]////////////////////////////////////////////////////////////////////
//Global variables should be initialised in robot_setup.c, then an extern to them should be placed
//here, otherwise we end up with multiple definition errors.
extern RobotGlobalStructure sys;
//TODO: add these to the sys structure
extern volatile char streamDelayCounter;

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void robotSetup(void)
*
* The initialisation routine for all hardware in the robot.
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void robotSetup(void);

/*
* Function:
* void masterClockInit(void)
*
* Initialises the master clock to 100MHz. The master clock is the clock source that drives all the
* peripherals in the micro controller.
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void masterClockInit(void);

/*
* Function:
* uint8_t waitForFlag(uint32_t *regAddr, uint32_t regMask, uint16_t timeOutMs)
*
* Will wait for the given status bit to become true. If it doesn't become true in the time
* specified in timeOutMs, then the function exits with an error.
*
* Inputs:
* uint32_t *regAddr
*	The address to the status register that is to be monitored.
* uint32_t regMask
*   The bit mask to apply to the given register.
* uint16_t timeOutMs
*   The maximum number of milliseconds to wait before exiting the function with an error.
*
* Returns:
* 0 if flag was detected or 1 if timeout was reached before flag was detected.
*
*/
uint8_t waitForFlag(const volatile uint32_t *regAddr, uint32_t regMask, uint16_t timeOutMs);

/*
* Function:
* type capToRangeInt(type valueToCap, type minimumVal, type maximumVal)
*
* Will see if a value is within a given range. If it is outside the given range, then limit the
* value to the given minimum or maximum value. Three different versions of this function operate on
* different types of variable. (Signed and unsigned integers, and single precision floating point
* numbers.
*
* Inputs:
* valueToCap:
*   The number we are checking to see if it is in range.
* minimumVal:
*   The minimumValue that we would like valueToCap to be
* maximumVal:
*   The maximum value we would like valueToCap to be.
*
* Returns:
* If valueToCap was outside the desired range, then a range limited version of valueToCap is
* returned, otherwise valueToCap is returned unmodified.
*
*/
int32_t capToRangeInt(int32_t valueToCap, int32_t minimumVal, int32_t maximumVal);

uint32_t capToRangeUint(uint32_t valueToCap, uint32_t minimumVal, uint32_t maximumVal);

float capToRangeFlt(float valueToCap, float minimumVal, float maximumVal);

#endif /* ROBOTDEFINES_H_ */