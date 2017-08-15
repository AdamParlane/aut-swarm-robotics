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
* Functions for intialising PIO and setting up LEDs
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void funcName(void)
*
*/
///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "pio_interface.h"

///////////////Functions////////////////////////////////////////////////////////////////////////////
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
* void ledInit(void)
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
void ledInit(void)
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
	ledOff1;					//D1 starts up off
	ledOff2;					//D2 starts up off
	ledOff3;					//D3 starts up off
}