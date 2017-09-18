/*
 * pio_interface.c
 *
 * Created: 16/08/2017 8:47:11 AM
 *  Author: Matthew
 */ 
/*
* pio_interface.h
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 16/08/2017 8:47:11 AM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Functions for intialising PIO and setting up/controlling LEDs
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void funcName(void)
*
*/
//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "pio_interface.h"

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
* Implementation:
* As described above.
*
*/
void pioInit(void)
{
	REG_PMC_PCER0
	|=	(1<<ID_PIOA);	//Enable clock access to PIO controller A
	REG_PMC_PCER0
	|=	(1<<ID_PIOB);	//Enable clock access to PIO controller B
	REG_PMC_PCER0
	|=	(1<<ID_PIOC);	//Enable clock access to PIO controller C
	REG_PIOA_WPMR
	=	0x50494F00;		//Disable PIOA write protect
	REG_PIOB_WPMR
	=	0x50494F00;		//Disable PIOB write protect
	REG_PIOC_WPMR
	=	0x50494F00;		//Disable PIOC write protect
}

/*
* Function:
* void pioLedInit(void)
*
* Initialises the PIO pins needed to use the LEDs
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* - Allow PIO controller to use pins PA27, PA28 and PC8
* - Enable PA27, PA28 and PC8 for output
* - Switch all LEDs off by default
*
*/
void pioLedInit(void)
{
	REG_PIOA_PER
	|=	(1<<28)					//PIO control enabled for D1 (LEDA)
	|	(1<<27);				//PIO control enabled for D3 (LEDC)
	REG_PIOC_PER
	|=	(1<<8);					//PIO control enabled for D2 (LEDB)
	REG_PIOA_OER
	|=	(1<<28)					//D1 as output
	|	(1<<27);				//D3 as output
	REG_PIOC_OER
	|=	(1<<8);					//D2 as output
	led1Off;					//D1 starts up off
	led2Off;					//D2 starts up off
	led3Off;					//D3 starts up off
}

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
* Implementation:
* A simply state machine selects which LEDs to turn on and off based on number given.
* If number is out of range then it is ignored.
*
*/
void pioLedNumber(uint8_t numeral)
{
	switch(numeral)
	{
		case 0:
			led1Off;
			led2Off;
			led3Off;
		break;

		case 1:
			led1On;
			led2Off;
			led3Off;
		break;

		case 2:
			led1Off;
			led2On;
			led3Off;
		break;

		case 3:
			led1On;
			led2On;
			led3Off;
		break;

		case 4:
			led1Off;
			led2Off;
			led3On;
		break;

		case 5:
			led1On;
			led2Off;
			led3On;
		break;

		case 6:
			led1Off;
			led2On;
			led3On;
		break;

		case 7:
			led1On;
			led2On;
			led3On;
		break;
	}
}