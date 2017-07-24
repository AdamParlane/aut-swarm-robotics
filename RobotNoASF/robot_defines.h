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
* uint16_t adcRead(uint8_t channel)
*
*/

#ifndef ROBOTDEFINES_H_
#define ROBOTDEFINES_H_

///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "spi.h"
#include "sam.h"
#include "imu_interface.h"
#include "opt_interface.h"
#include "motor_driver.h"
#include "twimux_interface.h"
#include "fc_interface.h"
#include "prox_sens_interface.h"
#include "light_sens_interface.h"
#include "line_sens_interface.h"
#include "communication.h"
#include "testFunctions.h"

///////////////Defines//////////////////////////////////////////////////////////////////////////////
//LED control macros
#define	ledOff1 	(REG_PIOA_CODR |= (1<<28))
#define	ledOff2		(REG_PIOC_CODR |= (1<<8))
#define	ledOff3 	(REG_PIOA_CODR |= (1<<27))
#define	ledOn1 		(REG_PIOA_SODR |= (1<<28))
#define	ledOn2 		(REG_PIOC_SODR |= (1<<8))
#define	ledOn3 		(REG_PIOA_SODR |= (1<<27))

//Analogue to digital conversion
//	Macros
#define adcStartConv			(REG_ADC_CR |= ADC_CR_START)	//Start ADC conversion
#define adcEnableChan(value)	(REG_ADC_CHER = (1<<(value)))	//Enable ADC channel for conversion
#define adcDisableChan(value)	(REG_ADC_CHDR = (1<<(value)))	//Disable ADC channel
#define adcData					(REG_ADC_LCDR)					//Last sampled ADC value
#define adcDataReady			(REG_ADC_ISR & ADC_ISR_DRDY)	//ADC conversion complete
//	ADC channel defines
//		Line follower ADC channels version 1 robot
#if defined ROBOT_TARGET_V1
#define LF0_ADC_CH			13	// Far left
#define LF1_ADC_CH			15	// Center left
#define LF2_ADC_CH			0	// Center right
#define LF3_ADC_CH			8	// Far right
#endif
//		Line follower ADC channels version 2 robot
#if defined ROBOT_TARGET_V2
#define LF0_ADC_CH			13	// Far left
#define LF1_ADC_CH			15	// Center left
#define LF2_ADC_CH			0	// Center right
#define LF3_ADC_CH			7	// Far right
#endif
//		Fast charge chip ADC channels
#define FC_BATVOLT_ADC_CH	14	// Battery voltage level
#define FC_BATTEMP_ADC_CH	9	// Battery temperature

//Universal Asynchronous Receiver/Transmitter
#define TXRDY (REG_UART3_SR & UART_SR_TXRDY)		//UART TX READY flag

#if !defined ROBOT_TARGET_V1 && !defined ROBOT_TARGET_V2
#error  Robot version has not been set in compiler options. (set ROBOT_TARGET_V1 or ROBOT_TARGET_V2)
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
//structure to receive the command and interpret it to something useful
{
	char messageClass;
	char commandCode;
	char command[10];
};

///////////////Global variables/////////////////////////////////////////////////////////////////////
//used for test function calling
char newDataFlag; //used for test function probably temporary
enum ROBOT_STATES{TEST, TEST_ALL, MANUAL, FORMATION, DOCKING, IDLE}; //main loop functionality
char robotState ;

///////////////Functions////////////////////////////////////////////////////////////////////////////
/*
* Function:
* uint16_t adcRead(uint8_t channel)
*
* Will read the instantaneous value of the given analogue to digital converter channel.
*
* Inputs:
* uint8_t channel
*   Channel number of the desired ADC channel (0-15)
*
* Returns:
* 12bit value of the ADC channel in question (0-4095)
*
*/
uint16_t adcRead(uint8_t channel);

#endif /* ROBOTDEFINES_H_ */