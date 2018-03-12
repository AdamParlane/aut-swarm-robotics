/*
* camera_buffer_interface.h
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

#ifndef CAMERA_BUFFER_INTERFACE_H_
#define CAMERA_BUFFER_INTERFACE_H_

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
//Buffer PIO Pin definitions
// Buffer pin				SAM4 pin		Function			Type
#define WE					PIO_PC7			// Write Enable		Output
#define WRST				PIO_PA24		// Write Reset		Output
#define RCK					PIO_PA15		// Read Clock		Output
#define OE					PIO_PA11		// Output Enable	Output
#define RE					PIO_PA11		// Read Enable		Output
#define RRST				PIO_PA26		// Read Reset		Output

// TEMP: will only allow for a single capture
// Timer Counter Reset
#define ReadClockHigh		REG_PIOA_SODR |= RCK;
#define ReadClockLow		REG_PIOA_CODR |= RCK;

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
void camBufferInit(void);

void camBufferWriteStop(void);
void camBufferWriteStart(void);
void camBufferWriteReset(void);

void camBufferReadStop(void);
void camBufferReadStart(void);
void camBufferReadReset(void);

uint8_t camBufferReadByte(void);
uint8_t camBufferReadData(uint32_t startAddr, uint32_t endAddr, uint8_t *data);

void TC1_Handler();

#endif /* CAMERA_BUFFER_INTERFACE_H_ */