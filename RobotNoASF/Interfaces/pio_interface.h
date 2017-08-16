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
#include "../robot_defines.h"

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
//LED PIO port and pin definitions
#define LED_1_PORT	PIOA
#define LED_1_PIN	PIO_PA28
#define LED_2_PORT	PIOC
#define LED_2_PIN	PIO_PC8
#define LED_3_PORT	PIOA
#define LED_3_PIN	PIO_PA27
//LED control macros
#define	ledOff1 	(LED_1_PORT->PIO_CODR |= LED_1_PIN)
#define	ledOff2		(LED_2_PORT->PIO_CODR |= LED_2_PIN)
#define	ledOff3 	(LED_3_PORT->PIO_CODR |= LED_3_PIN)
#define	ledOn1 		(LED_1_PORT->PIO_SODR |= LED_1_PIN)
#define	ledOn2 		(LED_2_PORT->PIO_SODR |= LED_2_PIN)
#define	ledOn3 		(LED_3_PORT->PIO_SODR |= LED_3_PIN)
#define ledTog1		{if(LED_1_PORT->PIO_ODSR&LED_1_PIN) ledOff1; else ledOn1;}
#define ledTog2		{if(LED_2_PORT->PIO_ODSR&LED_2_PIN) ledOff2; else ledOn2;}
#define ledTog3		{if(LED_3_PORT->PIO_ODSR&LED_3_PIN) ledOff3; else ledOn3;}

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

#endif /* PIO_INTERFACE_H_ */