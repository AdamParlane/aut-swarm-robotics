/*
* pio_interface.h
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 16/08/2017 8:46:57 AM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Macros for controlling LEDs, PIO related function prototypes
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void funcName(void)
*
*/

#ifndef PIO_INTERFACE_H_
#define PIO_INTERFACE_H_
//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
//LED PIO port and pin definitions
#define LED_1_PORT	PIOA
#define LED_1_PIN	PIO_PA28
#define LED_2_PORT	PIOC
#define LED_2_PIN	PIO_PC8
#define LED_3_PORT	PIOA
#define LED_3_PIN	PIO_PA27

//LED control macros
#define	led1On 		(LED_1_PORT->PIO_SODR |= LED_1_PIN)
#define	led1Off 	(LED_1_PORT->PIO_CODR |= LED_1_PIN)
#define led1Tog		{if(LED_1_PORT->PIO_ODSR&LED_1_PIN) led1Off; else led1On;}
	
#define	led2On 		(LED_2_PORT->PIO_SODR |= LED_2_PIN)
#define	led2Off		(LED_2_PORT->PIO_CODR |= LED_2_PIN)
#define led2Tog		{if(LED_2_PORT->PIO_ODSR&LED_2_PIN) led2Off; else led2On;}
	
#define	led3On 		(LED_3_PORT->PIO_SODR |= LED_3_PIN)
#define	led3Off 	(LED_3_PORT->PIO_CODR |= LED_3_PIN)
#define led3Tog		{if(LED_3_PORT->PIO_ODSR&LED_3_PIN) led3Off; else led3On;}

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
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
* void pioLedInit(void)
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
void pioLedInit(void);

/*
* Function:
* void pioLedNumber(uint8_t numeral)
*
* Will display the given number in binary on the LEDs. Useful when debugging state machines (can
* see the number of the state you are in)
*
* Inputs:
* uint8_t numeral:
*   An unsigned integer between 0-7 to display
*
* Returns:
* none
*
*/
void pioLedNumber(uint8_t numeral);

#endif /* PIO_INTERFACE_H_ */