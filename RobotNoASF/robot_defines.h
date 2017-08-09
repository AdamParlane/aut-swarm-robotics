/*
* robot_defines.h
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
* void pioInit(void)
* void ledInit(void)
*
*/

#ifndef ROBOTDEFINES_H_
#define ROBOTDEFINES_H_

//TODO: change something so that this doesnt have to be first
//Or maybe all defines should be before indludes
enum ROBOT_STATES{TEST, TEST_ALL, MANUAL, FORMATION, DOCKING, OBSTACLE_AVOIDANCE, IDLE, CHARGING}; //main loop functionality

///////////////Type Definitions/////////////////////////////////////////////////////////////////////
struct Command
//is anyone using this??? not me -Matt
//structure to receive the command and interpret it to something useful
{
	char messageClass;
	char commandCode;
	char command[10];
};

struct Position
//structure to store all the robot side navigation / positioning data
//this will be written to by getMouseXY, getEulerAngles, and another navigation function which
//combines them. The structure will store the relevant info from both key sensors and fuse them in
//an additional function
{
	short opticalDX;
	short opticalDY;
	float opticalX;
	float opticalY;
	long imuQW;
	long imuQX;
	long imuQY;
	long imuQZ;
	short imuAccelX;
	short imuAccelY;
	short imuAccelZ;
	short imuGyroX;
	short imuGyroY;
	short imuGyroZ;
	double imuPitch;
	double imuRoll;
	double imuYaw;
	unsigned long imuTimeStamp;
	float x;
	float y;
	float h;
};

///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "Interfaces/spi.h"
#include "sam.h"
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

///////////////Defines//////////////////////////////////////////////////////////////////////////////
//LED control macros
#define	ledOff1 	(REG_PIOA_CODR |= (1<<28))
#define	ledOff2		(REG_PIOC_CODR |= (1<<8))
#define	ledOff3 	(REG_PIOA_CODR |= (1<<27))
#define	ledOn1 		(REG_PIOA_SODR |= (1<<28))
#define	ledOn2 		(REG_PIOC_SODR |= (1<<8))
#define	ledOn3 		(REG_PIOA_SODR |= (1<<27))

//Universal Asynchronous Receiver/Transmitter
#define TXRDY (REG_UART3_SR & UART_SR_TXRDY)	//UART TX READY flag [SHOULD BE IN COMMUNICATIONS]

//If robot target compiler symbol is not present, then throw an error.
#if !defined ROBOT_TARGET_V1 && !defined ROBOT_TARGET_V2
#error  Robot version has not been set in compiler symbols. (set ROBOT_TARGET_V1 or ROBOT_TARGET_V2)
#endif



///////////////Global variables/////////////////////////////////////////////////////////////////////
//used for test function calling
char newDataFlag; //used for test function probably temporary
char robotState, previousState;
volatile char streamDelayCounter, streamIntervalFlag;

///////////////Functions////////////////////////////////////////////////////////////////////////////
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
* void pioInit(void)
*
* Supplies master clock to the three parallel I/O controllers (A, B and C) and disables write
* protection on their configuration registers.
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void pioInit(void);

/*
* Function:
* void ledInit(void)
*
* Initialises the PIO pins needed to use the LEDs. pioInit() MUST be run first.
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void ledInit(void);

#endif /* ROBOTDEFINES_H_ */