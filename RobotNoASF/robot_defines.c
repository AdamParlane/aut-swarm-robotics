/*
* robot_defines.c
*
* Author : Adam Parlane, Matthew Witt
* Created: 6/7/2017
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Contains general/miscellaneous  functions
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

///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "robot_defines.h"

///////////////Global variables/////////////////////////////////////////////////////////////////////
extern uint32_t systemTimestamp;	//Required by waitForFlag()

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
* Implementation:
* - Flash Wait state must be altered to suit 100MHz master clock or else there are problems reading
*   and writing to flash.
* - Provide password to disable write protection in the PMC and clock generator registers
* - Set the start up time for the main crystal oscillator (the external 12MHz one) to 5ms. This is
*   how long the micro controller will wait before setting MOSCXTS or main crystal oscillator status
*   bit in REG_PMC_SR to 1. When it equals 1 it means the oscillator has stabilised.
* - Change the Main Clock source to the external 12MHz crystal oscillator and wait the time
*   specified earlier before continuing. The Main clock supplies a clock source to the Master clock,
*   The programmable clock controller and the phase-locked loops. The PLLs can also supply the
*   Master clock, which is what we will be doing to boost the external clock frequency from 12MHz
*   to 100MHz.
* - Disable the onboard RC oscillator which is enabled by default. No longer needed as we are now
*   running on the external crystal oscillator.
* - Set up PLLA to multiply the Main clock from 12MHz to 100MHz (12MHz/4*25 = 100MHz) and wait 63
*   slow clock cycles before continuing so that PLLA can stabilise.
* - Finally, set PLLA as the master clock source and wait for Master clock to stabilise before
*   continuing.
*
*/
void masterClockInit(void)
{
	REG_EFC_FMR
	=	(1<<26)					//Opcode loop optimisation enabled
	|	(5<<8);				//Set Flash Wait State for 100MHz (5 cycles for read write
	//operations to flash
	REG_PMC_WPMR
	=	0x504D4300;				//Disable PMC write protect
	REG_CKGR_MOR
	|=	(0x37<<16)				//Set 5ms main xtal osc. Start up time.
	|	(0x14<<8);				//Start Up Time = 8 * MOSCXTST / SLCK => MOSCXTST = 20
	REG_CKGR_MOR
	|=	(0x37<<16)				//Write enable
	|	(1<<0);					//Enable the external crystal connected to XIN and XOUT
	while(!(REG_PMC_SR & 0x01));//Wait for the main crystal oscillator to stabilize
	REG_CKGR_MOR
	|=	(0x37<<16)				//Write enable
	|	(1<<24);				//MAINCK source set to external xtal
	while(!(REG_PMC_SR & 0x10000));//Wait for the source changeover to be complete
	REG_CKGR_MOR
	=	0x01371401;				//Disable the RC oscillator
	REG_CKGR_PLLAR
	|=	(1<<29)					//Must be 1 as per datasheet (pg540)
	|	(3<<0)					//Divide by 3
	|	(24<<16)				//Multiply by 25 (24 + 1)
	|	(0x3F<<8);				//Wait 63 SCLK cycles before setting LOCKA bit in REG_PMC_SR
	while(!(REG_PMC_SR & 0x02));//Wait for PLL LOCKA bit to be set
	REG_PMC_MCKR
	=	(2<<0);					//Set PLLA_CLK as MCK
	while(!(REG_PMC_SR & 0x08));//Wait for MCK ready
}

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
*   The bitmask to apply to the given register.
* uint16_t timeOutMs
*   The maximum number of milliseconds to wait before exiting the function with an error.
*
* Returns:
* 0 if flag was detected or 1 if timeout was reached before flag was detected.
*
* Implementation:
* - System timestamp is loaded into a variable so we know at what time this function started.
* - A while function then waits for the flag to be set in the given register. It is also checking
*   if the time out period has been reached. The while loop exits when the flag is set or the
*   timeout period expires.
* - If the while loop exited because the flag was set, exit the function with a 0 value, otherwise
*   exit the function with a 1 (indicating an error)
*
*/
uint8_t waitForFlag(const volatile uint32_t* regAddr, uint32_t regMask, uint16_t timeOutMs)
{
	uint32_t startTime = systemTimestamp;
	while(!((*regAddr) & regMask) && (systemTimestamp < (startTime + timeOutMs)));
	if((*regAddr) & regMask)
		return 0;
	else
		return 1;
}