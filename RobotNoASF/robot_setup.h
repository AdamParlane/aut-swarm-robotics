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

//TODO: change something so that this doesn't have to be first
//Or maybe all defines should be before includes
enum ROBOT_STATES{TEST, TEST_ALL, MANUAL, FORMATION, DOCKING, OBSTACLE_AVOIDANCE, IDLE, CHARGING, LINE_FOLLOW, LIGHT_FOLLOW}; //main loop functionality

///////////////Type Definitions/////////////////////////////////////////////////////////////////////
struct Position
//structure to store all the robot side navigation / positioning data
//this will be written to by getMouseXY, nfGetEulerAngles, and another navigation function which
//combines them. The structure will store the relevant info from both key sensors and fuse them in
//an additional function (84bytes i think)
{
	unsigned short opticalDX;		//Rate of change from optical sensor (X axis is left to right)
	unsigned short opticalDY;		//Rate of change from optical sensor (Y axis is fwd/bckwd)
	float opticalX;
	float opticalY;
	float opticalHdg;		//Heading calculated from optical sensor
	float opticalSpeed;		//Magnitude calculated from optical sensor
	long imuQW;				//W component of the quaternion complex number returned by DMP
	long imuQX;				//X component of the quaternion complex number returned by DMP
	long imuQY;				//Y component of the quaternion complex number returned by DMP
	long imuQZ;				//Z component of the quaternion complex number returned by DMP
	float imuAccelX;		//Delta X Acceleration in ms^2
	float imuAccelY;		//Delta Y Acceleration in ms^2
	float imuAccelZ;		//Delta Z Acceleration in ms^2
	float imuGyroX;			//Delta pitch in deg/s
	float imuGyroY;			//Delta roll in	deg/s
	float imuGyroZ;			//Delta yaw in deg/s (Delta heading)
	float imuPitch;			//Absolute pitch from DMP (degrees)
	float imuRoll;			//Absolute roll from DMP (degrees)
	float imuYaw;			//Absolute yaw (heading) from DMP (degrees)
	float imuYawOffset;		//Used to offset heading value (when corrected by PC)
	unsigned long imuTimeStamp;//Time at which last IMU reading took place (ms)
	unsigned short imuDeltaTime;//Time between last IMU reading and IMU previous reading
	float x;				//Absolute X position in arena
	float y;				//Absolute Y position in arena
	float h;				//Absolute Z position in arena
};

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "Interfaces/spi.h"
#include "sam.h"
#include "Interfaces/pio_interface.h"
#include "Interfaces/imu_interface.h"
#include "Interfaces/timer0.h"
#include "Interfaces/external_interrupt.h"
#include "Interfaces/communication.h"
#include "Interfaces/adc_interface.h"
#include "Interfaces/opt_interface.h"
#include "Interfaces/motor_driver.h"
#include "Interfaces/twimux_interface.h"
#include "Interfaces/fc_interface.h"
#include "Interfaces/prox_sens_interface.h"
#include "Interfaces/light_sens_interface.h"
#include "Interfaces/line_sens_interface.h"
#include "Functions/testFunctions.h"
#include "Functions/docking_functions.h"
#include "Functions/manual_mode.h"
#include "Functions/obstacle_avoidance.h"
#include "Functions/motion_functions.h"
#include "Functions/navigation_functions.h"

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
	
//Universal Asynchronous Receiver/Transmitter
#define TXRDY (REG_UART3_SR & UART_SR_TXRDY)	//UART TX READY flag [SHOULD BE IN COMMUNICATIONS]

//If robot target compiler symbol is not present, then throw an error.
#if !defined ROBOT_TARGET_V1 && !defined ROBOT_TARGET_V2
#error  Robot version has not been set in compiler symbols. (set ROBOT_TARGET_V1 or ROBOT_TARGET_V2)
#endif

//////////////[Global variables]////////////////////////////////////////////////////////////////////
//used for test function calling
char newDataFlag; //TODO:used for test function probably temporary ((still temporary?)
char mainRobotState, mainRobotStatePrev;	//main function state machine states
volatile char streamDelayCounter, streamIntervalFlag;	//TODO:What are these?
signed int aim;
char aimSpeed;
char movingFlag;
char tMit;
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