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

///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "spi.h"
#include "sam.h"
#include "communication.h"
#include "adc_interface.h"
#include "imu_interface.h"
#include "opt_interface.h"
#include "motor_driver.h"
#include "twimux_interface.h"
#include "fc_interface.h"
#include "prox_sens_interface.h"
#include "light_sens_interface.h"
#include "line_sens_interface.h"
#include "testFunctions.h"
#include "docking_functions.h"
#include "manual_mode.h"

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

///////////////Type Definitions/////////////////////////////////////////////////////////////////////
struct Position
//struture to store all the robot side navigation / positioning data
//this will be written to by getMouseXY, getEulerAngles, and another navigation function which combines them
//The struture will store the relevant info from both key sensors and fuse them in an additional function
{
	uint16_t opticaldx;
	uint16_t opticaldy;
	float opticalx;
	float opticaly;
	float IMUqw;
	float IMUqx;
	float IMUqy;
	float IMUqz;
	float x;
	float y;
	float h;
};

struct Command
//is anyone using this???
//structure to receive the command and interpret it to something useful
{
	char messageClass;
	char commandCode;
	char command[10];
};

struct transmitDataStructure
{
	uint8_t Data[50];//array for data to be transmitted to PC BEFORE XBee framing has been added
	uint8_t DataSize;//size of the transmit array
};

///////////////Global variables/////////////////////////////////////////////////////////////////////
//used for test function calling
char newDataFlag; //used for test function probably temporary
enum ROBOT_STATES{TEST, TEST_ALL, MANUAL, FORMATION, DOCKING, IDLE, PAUSE}; //main loop functionality
char robotState;
char streamDelayCounter, streamIntervalFlag;


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