/*
* camera_buffer_interface.c
*
* Author : Brae HW (bhw11@hotmail.co.nz)
* Created: 27/07/2017 1:51:11 PM
* 
* Project Repository: https://github.com/wittsend/aut-swarm-robotics
*
* Header file for the camera memory buffer interface.
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void camBufferInit(void);
* void camBufferWriteStop(void);
* void camBufferWriteStart(void);
* void camBufferWriteReset(void);
* void camBufferReadStop(void);
* void camBufferReadStart(void);
* void camBufferReadReset(void);
* uint8_t camBufferReadByte(void);
*
*/ 

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "camera_interface.h"
#include "camera_buffer_interface.h"
#include "timer_interface.h"	//Provides delay_ms()

//////////////[Private Defines]/////////////////////////////////////////////////////////////////////

#define DO0					(REG_PIOA_PDSR & PIO_PA6)
#define DO1					(REG_PIOA_PDSR & PIO_PA9)
#define DO2					(REG_PIOA_PDSR & PIO_PA10)
#define DO3					(REG_PIOC_PDSR & PIO_PC2)
#define DO4					(REG_PIOA_PDSR & PIO_PA25)
#define DO5					(REG_PIOC_PDSR & PIO_PC4)
#define DO6					(REG_PIOC_PDSR & PIO_PC5)
#define DO7					(REG_PIOC_PDSR & PIO_PC6)

// Writing to buffer from camera
#define WriteDisable		REG_PIOC_SODR |= WE				// Buffer write enable
#define WriteEnable			REG_PIOC_CODR |= WE				// Buffer write disable
#define WriteReset			REG_PIOA_CODR |= WRST			// Buffer write reset
#define WriteOn				REG_PIOA_SODR |= WRST			// Buffer write on (not in reset)

// Reading from buffer to SAM4
#define readResetEnable		REG_PIOA_CODR	|= RRST			// Buffer read reset
#define readResetDisable	REG_PIOA_SODR	|= RRST			// Buffer read on (not in reset)
#define OutputEnable		REG_PIOA_CODR	|= OE			// Buffer output enable
#define OutputDisable		REG_PIOA_SODR	|= OE			// Buffer output disable
#define readClockEnable		REG_TC0_CCR1	|= TC_CCR_CLKEN|TC_CCR_SWTRG//Enable the read clock
#define readClockDisable	REG_TC0_CCR1	= 0				//Disable the read clock
#define readNow				REG_TC0_SR1		& TC_SR_CPCS	//A flag that says whether RC has over-
															//flowed on TC1. (Indicates when to read
															//a byte from the RAM)

//////////////[Private Global Variables]////////////////////////////////////////////////////////////
volatile uint32_t ramAddrPointer = 0;//Indicates the address in buffer that is currently being read
//TODO: Make sure that this variable is reset to 0 everytime read reset is asserted to stay synced
//with the buffer.

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
// TEMP: Dirty Delay Design TODO: Replace with existing delay function
void delayBuffer()
{
	// Dirty: variable
	int x = 0;
	// Delay: Not even sure
	while (x < 100000) x++;
	// Design: 3/10
	return;
}

void camBufferInit()
{
	/********Buffer control**************/
	//buffer AL422B can store one complete VGA frame

	/************Write to buffer from Camera**********/
	// buffer AL422B has a active low WE write enable. a 0 is applied to this pin from NAND gate
	//U7 when HREF (camera) and VB_WE (micro) is high
	// this means the buffer only captures 640 pixels active pixels in one line.
	
	//Set up the IO pins for controlling the buffer chip
	REG_PIOA_PER			//Peripheral enable
	|=	WRST				//PA24 (VB_WRST)
	|	WE;					//PC7 (VB_WE)
	REG_PIOA_OER			//Enable pins as outputs
	|=	WRST				//PA24 (VB_WRST)
	|	WE;					//PC7 (VB_WE)
	REG_PIOA_PUER			//Enable internal pullup resistors
	|=	WRST				//PA24 (VB_WRST)
	|	WE;					//PC7 (VB_WE)

	/***********Read from buffer to micro***********/
	// the buffer must have read enable (RE - tied to gnd), Output enable (OE) and read reset(RRST)
	//low in order to output stored data
	REG_PIOA_PER			//Enable PIO control for buffer
	|=	RRST				//PA26 (VB_RRST)
	//|	RCK					//PA15 (Read Clock)
	|	OE;					//PA11 (VB_OE)
	REG_PIOA_OER			//Set as an output
	|=	RRST				//PA26 (VB_RRST)
	|	RCK					//PA15 (Read clock)
	|	OE;					//PA11 (VB_OE)
	REG_PIOA_PUER			//Enable internal pullup resistors
	|=	RRST				//PA26 (VB_RRST)
	|	RCK					//PA15 (Read clock)
	|	OE;					//PA11 (VB_OE)


	//TIMER for READ CLOCK
	//Timer Counter 0 Channel 1 Config (Used for the camera buffer read clock RCK on PA15 (TIOA1)
	//Enable the peripheral clock for TC0
	//Enable interrupts
	//NVIC_EnableIRQ(ID_TC1);				//Enable interrupts on Timer Counter 0 Channel 1
	REG_PMC_PCER0
	|=	(1<<ID_TC1);
	REG_TC0_CMR1						//TC Channel Mode Register (Pg877)
	|=	TC_CMR_TCCLKS_TIMER_CLOCK1		//Prescaler MCK/2 (100MHz/2 = 50MHz)
	|	TC_CMR_WAVE						//Waveform mode
	|	TC_CMR_WAVSEL_UP_RC				//Up mode with auto triggering on RC compare
	|	TC_CMR_ACPA_CLEAR				//Clear TIOA1 on RA compare
	|	TC_CMR_ACPC_SET;				//Set TIOA1 on RC compare (Read data on rising edge)
	REG_TC0_RA1							//RA set to 12 counts
	|=	(TC_RA_RA(12));
	REG_TC0_RC1							//RC set to 25 counts (total (almost) square wave of 500ns
	|=	(TC_RC_RC(25));					//period, 2MHZ read clock)
	REG_TC0_IER1						//TC interrupt enable register
	|=	TC_IER_CPCS;					//Enable Register C compare interrupt
	REG_TC0_CCR1						//Clock control register
	=	0;								//Keep the timer disabled until its needed
	
	REG_PIOA_ABCDSR1
	|=	(PIO_ABCDSR_P15);				//Set PA15 for peripheral B (TIOA1)
	REG_PIOA_PDR
	|=	RCK;							//Allow TC1 to use RCK (PA15)

	/******* Micro camera data lines**************/
	/* 8 data lines coming into the micro from the buffer as a byte of half a pixel*/
	REG_PIOC_PER			//Enable PIO control
	|=	PIO_PC2				//D3(PC2), 
	|	PIO_PC4				//D5(PC4),
	|	PIO_PC5				//D6(PC5),
	|	PIO_PC6;  			//D7(PC6)
	REG_PIOC_ODR 			//Set D3,D5,D6,D7 as an input
	|=	PIO_PC2
	|	PIO_PC4 
	|	PIO_PC5 
	|	PIO_PC6; 
	REG_PIOC_PUER 			//Enable internal pullups
	|=	PIO_PC2
	|	PIO_PC4
	|	PIO_PC5
	|	PIO_PC6;
	REG_PIOA_PER			//Enable PIO control
	|=	PIO_PA6 			//D0(PA6),
	|	PIO_PA9				//D1 (PA9),
	|	PIO_PA10			//D2(PA10),
	|	PIO_PA24;			//D4(PA24)
	REG_PIOA_ODR			//Set D0,D1,D2,D4 as an input
	|=	PIO_PA6 
	|	PIO_PA9
	|	PIO_PA10
	|	PIO_PA24;
	REG_PIOA_PUER			//Enable internal pullup resistors
	|=	PIO_PA6
	|	PIO_PA9
	|	PIO_PA10
	|	PIO_PA24;
		
	//100ms after power, the buffer chip should be reset (Pg14 of datasheet):
	delay_ms(100);
	camBufferWriteReset();
	camBufferReadReset();
}

void camBufferWriteStop(void)
{
	WriteDisable;
}

void camBufferWriteStart(void)
{
	// Start write
	WriteEnable;
}

void camBufferWriteReset(void)
{
	// Reset
	WriteReset;
	delay_ms(1);
	//delayBuffer();
	// Clear reset
	WriteOn;
	//delayBuffer();
}

void camBufferReadStop(void)
{
	readClockDisable;
	OutputDisable;
	delay_ms(1);
}

void camBufferReadStart(void)
{
	OutputEnable;
	delay_ms(1);
	readClockEnable;
}

void camBufferReadReset(void)
{
	// Reset
	readResetEnable;
	delay_ms(1);	//TODO:Shorter delays would be better in future
	// Clear reset
	readResetDisable;
	ramAddrPointer = 0;
}

uint8_t camBufferReadByte(void)
{
	uint8_t d0,d1,d2,d3,d4,d5,d6,d7;
	
	d0 = DO0 ? 0x01 : 0x00;
	d1 = DO1 ? 0x02 : 0x00;
	d2 = DO2 ? 0x04 : 0x00;
	d3 = DO3 ? 0x08 : 0x00;
	d4 = DO4 ? 0x10 : 0x00;
	d5 = DO5 ? 0x20 : 0x00;
	d6 = DO6 ? 0x40 : 0x00;
	d7 = DO7 ? 0x80 : 0x00;
	
	//d0 = DO0 ? 0x80 : 0x00;
	//d1 = DO1 ? 0x40 : 0x00;
	//d2 = DO2 ? 0x20 : 0x00;
	//d3 = DO3 ? 0x10 : 0x00;
	//d4 = DO4 ? 0x08 : 0x00;
	//d5 = DO5 ? 0x04 : 0x00;
	//d6 = DO6 ? 0x02 : 0x00;
	//d7 = DO7 ? 0x01 : 0x00;
	
	return (d0|d1|d2|d3|d4|d5|d6|d7);

}

/*
* Function:
* uint8_t camBufferReadData(uint32_t startAddr, uint32_t endAddr, uint16_t *data)
*
* Allows the caller to read the data between two addresses in the video RAM buffer
*
* Inputs:
* unint32_t startAddr:
*	The address to start reading from the buffer
* uint32_t endAddr:
*	The address to finish reading from the data
* uint16_t *data:
*	Pointer to an array that is large enough to store the retrieved data.
*
* Returns:
* 0 on success
*
* Implementation:
* [explain key steps of function]
* [use heavy detail for anything complicated]
* Template c file function header. H file function header will be the same without the
* implementation/improvement section
*
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
uint8_t camBufferReadData(uint32_t startAddr, uint32_t endAddr, uint8_t *data)
{
	uint32_t readWriteDiff = 0;
	
	//If the ramAddrPointer is greater than the startAddr, then reset the RAM's read pointer
	if(startAddr < ramAddrPointer)
	{
		camBufferReadStop();	//Make sure we aren't already in read mode
		camBufferReadReset();	//Reset the read pointers
	}
	
	readWriteDiff = ramAddrPointer;
	
	//Start the read clock
	camBufferReadStart();
	
	//Wait for the RAM read pointer to reach the start address
	while(ramAddrPointer < startAddr);
	
	//Now we can begin pulling data from the RAM
	while(ramAddrPointer >= startAddr && ramAddrPointer <= endAddr)
	{
		
		if(readNow)		//readNow is reset when it is read from.
		{
			//We want to be reading on the rising edge of the read clock
			ramAddrPointer++;
			data[ramAddrPointer - readWriteDiff] = camBufferReadByte();
		}
	}
	
	camBufferReadStop();
	
	return 0;
}

void TC1_Handler()
{
	ramAddrPointer++;
}