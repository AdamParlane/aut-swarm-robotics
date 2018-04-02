/*
* robot_setup.h
*
* Author : Adam Parlane, Matthew Witt
* Created: 6/7/2017
*
* Project Repository: https://github.com/wittsend/aut-swarm-robotics
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
	M_OBSTACLE_AVOIDANCE_DEMO,
	M_IDLE, 
	M_CHARGING,
	M_LINE_FOLLOW,
	M_LIGHT_FOLLOW, 
	M_RANDOM,
	M_MOVE_TO_POSITION,
	M_STARTUP_DELAY
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
	FLS_ALIGN,
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
	int dxSum;			//Sum of samples between updates from optical sensor (for rolling average)
	int dySum;			//Sum of samples between updates from optical sensor (for rolling average)
	uint16_t sampleCount;	//Number of samples between updates (for rolling average)
	int x;				//Count sum on x axis
	int y;				//Count sum on y axis
	int xOld;			//x value at the last PC update
	int yOld;			//y value at the last PC update
	char pollEnabled;	//Enable polling the optical sensor
	char pollInterval;	//Rate at which to poll Mouse
	char overflowFlag;	//1 if data has overflowed on optical sensor
	uint8_t surfaceQuality;	//A value signifying quality of the surface (242 = max quality)
	float convFactor;	//A coefficient to convert to mm
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
typedef struct PositionGroup
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
	int32_t oldPCX;				//The last X position from the PC
	int32_t oldPCY;				//The last Y position from the PC
	int32_t targetX;			//The target x position in mm from the PC
	int32_t targetY;			//The target y position in mm from the PC
} PositionGroup;

//Stores information about the battery and charging
typedef struct BatteryChargeData
{
	uint16_t batteryVoltage;			//Battery voltage in mV
	uint8_t batteryTemp;				//Battery temperature in degrees
	uint16_t batteryMaxVoltage;			//Fully charged voltage of battery (will calibrate onthefly)
	uint16_t batteryDockingVoltage;		//Voltage below which the robot should seek dock
	uint16_t batteryMinVoltage;			//Voltage at which robot is considered completely dead
	uint8_t batteryPercentage;			//Percentage of battery remaining
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
	uint16_t red;
	uint16_t green;
	uint16_t blue;
	uint16_t white;
	uint16_t hue;
	uint16_t saturation;
	uint16_t value;
} ColourSensorData;

//Stores proximity data
typedef struct ProximityData
{
	uint8_t errorCount;					//Counts the number of times proximity sensors have failed
	uint8_t pollEnabled;				//Whether or not to poll for new messages in main()
	uint16_t pollInterval;				//Interval at which to poll at (ms)
} ProximityData;

//Will store states of the line sensors. This is necessary
//because there is a gray area when the sensor is half on and half off the line, so by establishing
//hysteresis and only changing the stored states when an upper and lower threshold is crossed,
//jitter should be reduced.
typedef struct LineSensorArray
{
	uint8_t outerLeft;
	uint8_t innerLeft;
	uint8_t innerRight;
	uint8_t outerRight;
	uint16_t pollInterval;
	uint8_t pollEnabled;
	int8_t direction;
	uint8_t detected;
} LineSensorArray;

struct transmitDataStructure
{
	char Data[50];//array for data to be transmitted to PC BEFORE XBee framing has been added
	uint8_t DataSize;//size of the transmit array
};

typedef struct CommunicationDataGroup
{
	uint8_t pollEnabled;				//Whether or not to poll for new messages in main()
	uint8_t twi2SlavePollEnabled;		//Whether to look for slave requests on twi2 (From LCD)
	uint8_t twi2ReceivedDataByte;		//Stores the last received data byte from TWI2 slave
	uint16_t pollInterval;				//Interval at which to poll at (ms)
	uint16_t pcUpdateInterval;			//Interval at which the PC is updated with the robots status
	uint8_t pcUpdateEnable;
	struct transmitDataStructure transmitData;
	struct MessageInfo messageData;		//Next message data
	uint16_t testModeStreamInterval;	//Interval between sending test data packets (ms)
} CommunicationDataGroup;

//Proximity sensor sub-structure
typedef struct ProximitySensorGroup
{
	uint16_t sensor[6];
	uint8_t pollEnabled;	//Bitmask of the sensors being polled
	uint16_t pollInterval;
	uint8_t errorCount;
}ProximitySensorGroup;

//Structure that will store all system flags for global use
typedef struct SystemFlagsGroup
{
	char xbeeNewData;	//New data from Xbee interface
	char imuCheckFifo;	//IMU ext interrupt has been triggered
	char camBufferRead;	//A new image is ready to be read from the camera FIFO buffer
	char twi2NewData;	//New data received on twi2 (Slave interface)
	char obaMoving;		//Robot is in motion
	char obaEnabled;	//Obstacle avoidance enabled
	char cornerFlag;
	char posPCNewData;	//New position information from the PC
} SystemFlagsGroup;

//Structure that will store the state of every state machine in the system
typedef struct SystemStatesGroup
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
} SystemStatesGroup;

//Colour sensor sub structure
typedef struct ColourSensorGroup
{
	ColourSensorData left;
	ColourSensorData right;
	uint16_t pollInterval;		//Poll rate
	uint8_t pollEnabled;		//Contains a bitmask indicating which sensors are updated
	uint8_t getHSV;				//Whether or not to convert to HSV when retrieving
} ColourSensorGroup;

//Camera sensor sub structure
typedef struct CameraSensorGroup
{
	bool initialised;	//flag to indicate if the camera has been successfully initialised
} CameraSensorGroup;

//Sensors sub-structure
typedef struct SensorDataGroup
{
	LineSensorArray line;
	ColourSensorGroup colour;
	ProximitySensorGroup prox;
	CameraSensorGroup camera;
} SensorDataGroup;

//Root Structure to combine all system globals
typedef struct RobotGlobalStructure
{
	SystemStatesGroup states;				//System states
	SystemFlagsGroup flags;					//System global flags
	SensorDataGroup sensors;				//Sensor data
	CommunicationDataGroup comms;			//Communication system control and data
	PositionGroup pos;						//Position information
	BatteryChargeData power;				//Battery/Charging info and control
	uint32_t timeStamp;						//System timestamp (millisecs since power on)
	uint16_t startupDelay;					//Time to wait between sys setup and execution
} RobotGlobalStructure;

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
//Fixes an issue in Atmel's CMSIS implementation
#define REG_PIOA_ABCDSR1 (*(__IO uint32_t*)0x400E0E70U)
#define REG_PIOA_ABCDSR2 (*(__IO uint32_t*)0x400E0E74U)

//////////////[Global variables]////////////////////////////////////////////////////////////////////
//Global variables should be initialised in robot_setup.c, then an extern to them should be placed
//here, otherwise we end up with multiple definition errors.
extern RobotGlobalStructure sys;

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

//Convert double to string
char * dtoa(char *s, double n);

#endif /* ROBOTDEFINES_H_ */