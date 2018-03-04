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
* void CameraBufferInit(void);
* void CameraBufferWriteStop(void);
* void CameraBufferWriteStart(void);
* void CameraBufferWriteReset(void);
* void CameraBufferReadStop(void);
* void CameraBufferReadStart(void);
* void CameraBufferReadReset(void);
* uint8_t CameraBufferReadData(void);
*
*/ 

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "camera_interface.h"
#include "camera_buffer_interface.h"

//////////////[Private Defines]/////////////////////////////////////////////////////////////////////
//Fixes an issue in Atmel's CMSIS implementation
#define REG_PIOA_ABCDSR1 (*(__IO uint32_t*)0x400E0E70U)
#define REG_PIOA_ABCDSR2 (*(__IO uint32_t*)0x400E0E74U)

// Buffer pin				SAM4 pin		Function			Type
#define WE					PIO_PC7			// Write Enable		Output
#define WRST				PIO_PA24		// Write Reset		Output
#define RCK					PIO_PA15		// Read Clock		Output
#define OE					PIO_PA11		// Output Enable	Output
#define RE					PIO_PA11		// Read Enable		Output
#define RRST				PIO_PA26		// Read Reset		Output

// Data output pin			Macro
//#define DO0				((REG_PIOA_PDSR & PIO_PA6) ? 0x01 : 0x00)
//#define DO1				((REG_PIOA_PDSR & PIO_PA9) ? 0x02 : 0x00)
//#define DO2				((REG_PIOA_PDSR & PIO_PA10) ? 0x04 : 0x00)
//#define DO3				((REG_PIOC_PDSR & PIO_PC2) ? 0x08 : 0x00)
//#define DO4				((REG_PIOA_PDSR & PIO_PA25) ? 0x10 : 0x00)
//#define DO5				((REG_PIOC_PDSR & PIO_PC4) ? 0x20 : 0x00)
//#define DO6				((REG_PIOC_PDSR & PIO_PC5) ? 0x40 : 0x00)
//#define DO7				((REG_PIOC_PDSR & PIO_PC6) ? 0x80 : 0x00)

#define DO0					(REG_PIOA_PDSR & PIO_PA6)
#define DO1					(REG_PIOA_PDSR & PIO_PA9)
#define DO2					(REG_PIOA_PDSR & PIO_PA10)
#define DO3					(REG_PIOC_PDSR & PIO_PC2)
#define DO4					(REG_PIOA_PDSR & PIO_PA25)
#define DO5					(REG_PIOC_PDSR & PIO_PC4)
#define DO6					(REG_PIOC_PDSR & PIO_PC5)
#define DO7					(REG_PIOC_PDSR & PIO_PC6)

// Data output bytes
#define DataOut				(DO0 | DO1 | DO2 | DO3 | DO4 | DO5 | DO6 | DO7)
#define DataOut1			(DO0 | DO1 | DO2 | DO4)
#define DataOut2			(DO3 | DO5 | DO6 | DO7)

// Writing to buffer from camera
#define WriteEnable			REG_PIOC_SODR |= WE;			// Buffer write enable
#define WriteDisable		REG_PIOC_CODR |= WE;			// Buffer write disable
#define WriteReset			REG_PIOA_CODR |= WRST;			// Buffer write reset
#define WriteOn				REG_PIOA_SODR |= WRST;			// Buffer write on (not in reset)

// Reading from buffer to SAM4
#define ReadReset			REG_PIOA_CODR |= RRST;			// Buffer read reset
#define ReadOn				REG_PIOA_SODR |= RRST;			// Buffer read on (not in reset)
#define OutputEnable		REG_PIOA_CODR |= OE;			// Buffer output enable
#define OutputDisable		REG_PIOA_SODR |= OE;			// Buffer output disable

// TEMP: will only allow for a single capture
// Timer Counter Reset
#define ReadClockReset		REG_TC0_CCR1 |= TC_CCR_SWTRG;
#define ReadClockHigh		REG_PIOA_SODR |= RCK;
#define ReadClockLow		REG_PIOA_CODR |= RCK;

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

void CameraBufferInit()
{
	/********Buffer control**************/
	//buffer AL422B can store one complete VGA frame

	/************Write to buffer from Camera**********/
	// buffer AL422B has a active low WE write enable. a 0 is applied to this pin from NAND gate
	//U7 when HREF (camera) and VB_WE (micro) is high
	// this means the buffer only captures 640 pixels active pixels in one line.
	
	//Set up the IO pins for controlling the buffer chip
	REG_PIOA_PER |= WRST;	//Enable PIO control of PA24 (VB_WRST) write reset
	REG_PIOA_OER |= WRST;	//Set PA24 (VB_WRST) as an output
	REG_PIOC_PER |= WE;		//Enable PIO control of PC7 (VB_WE) write enable
	REG_PIOC_OER |= WE;		//Set PC7 (VB_WE) as an output

	/***********Read from buffer to micro***********/
	// the buffer must have read enable (RE - tied to gnd), Output enable (OE) and read reset(RRST)
	//low in order to output stored data
	REG_PIOA_PER			//Enable PIO control for buffer
	|=	RRST				//PA26 (VB_RRST)
	|	RCK					//PA15 (Read Clock)
	|	OE;					//PA11 (VB_OE)
	REG_PIOA_OER			//Set as an output
	|=	RRST				//PA26 (VB_RRST)
	|	RCK					//PA15 (Read Clock)
	|	OE;					//PA11 (VB_OE)

	/***********Timer for buffer read clock*****************/
	/* RCK is on PA15 which is timer 0 TC0 and channel 1 TIOA1.the peripheral clock
	has been set to timer 0 in the XCLK setup above*/
	// Select peripheral B
	//REG_PIOA_ABCDSR1 |= (PIO_ABCDSR_P15);
	//REG_PMC_PCER0 |= (1<<ID_TC1); 					// Enable peripheral clock on Timer0
	//REG_PIOA_PDR |= RCK;							// Enable TCO control of PA15
	//REG_TC0_CCR1 |= TC_CCR_CLKEN;					// Enable the channel 1 clock
	//REG_TC0_CMR1 |= TC_CMR_TCCLKS_TIMER_CLOCK1;		// Clk set to MCK/2 (100/2 = 50Mhz/20ns)
	//REG_TC0_CMR1 |= TC_CMR_WAVSEL_UP_RC;			// Up count cleared on RC compare
	//REG_TC0_CMR1 |= TC_CMR_WAVE;					// Waveform mode
	//REG_TC0_CMR1 |= TC_CMR_ACPA_SET;				// Set pulse on RA compare
	//REG_TC0_CMR1 |= TC_CMR_ACPC_CLEAR;				// Clear pulse on RC compare
	//REG_TC0_RA1 |= TC_RA_RA(25);					// RA set to 20 counts
	//REG_TC0_RC1 |= TC_RC_RC(50);					// RC set to 40 counts (total square wave of 
													// 800ns period, 1.25MHZ)
	//uint32_t temp = REG_TC0_SR1;					// Read ISR so that it clears any interrupt 
													// flags that might be there
	//REG_TC0_IER1 |= TC_IER_CPCS;					// Enable input change interrupt on RA compare
	//REG_TC0_CCR1 |= TC_CCR_SWTRG;

	//NVIC_EnableIRQ(TC1_IRQn);						// Enable interrupt vector for TIMER0

	/******* Micro camera data lines**************/
	/* 8 data lines coming into the micro from the buffer as a byte of half a pixel*/
	REG_PIOC_PER			//Enable PIO control
	|=	PIO_PC2				//D3(PC2), 
	|	PIO_PC4				//D5(PC4),
	|	PIO_PC5				//D6(PC5),
	|	PIO_PC6);  			//D7(PC6)
	REG_PIOC_ODR 			//Set D3,D5,D6,D7 as an input
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
	|	PIO_PA24);
}

void CameraBufferWriteStop(void)
{
	WriteDisable;
}

void CameraBufferWriteStart(void)
{
	// Start write
	WriteEnable;
}

void CameraBufferWriteReset(void)
{
	// Reset
	WriteReset;
	delayBuffer();
	// Clear reset
	WriteOn;
	delayBuffer();
}

void CameraBufferReadStop(void)
{
	OutputDisable;
	delayBuffer();
}

void CameraBufferReadStart(void)
{
	ReadClockReset;
	delayBuffer();
	// Start write
	OutputEnable;
	delayBuffer();
}

void CameraBufferReadReset(void)
{
	// Reset
	ReadReset;
	delayBuffer();
	// Clear reset
	ReadOn;
	delayBuffer();
}

uint8_t CameraBufferReadData(void)
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
	
	return (d0|d1|d2|d3|d4|d5|d6|d7);
}