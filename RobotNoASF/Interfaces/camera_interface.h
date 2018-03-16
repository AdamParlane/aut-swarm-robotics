/*
* camera_interface.h
*
* Author : Brae HW (bhw11@hotmail.co.nz)
* Created: 3/04/2017 5:08:53 PM
* 
* Project Repository: https://github.com/wittsend/aut-swarm-robotics
*
* Header file for the camera interface module.
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void camInit(void);
* uint8_t camSetup(void);
* void camWriteInstruction(uint8_t regAddress, uint8_t data);
* uint8_t camReadInstruction(uint8_t data);
* void camHardReset(void);
* void camRegisterReset(void);
* bool camValidID(void)
* void camChangeFormat(uint8_t type);
* void camTestPattern(CameraTestPatterns type);
* void camRead(void);
*
*/

#ifndef CAMERA_INTERFACE_H_
#define CAMERA_INTERFACE_H_

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"

//////////////[Public Defines]//////////////////////////////////////////////////////////////////////
//This has been made available publicly so the buffer can time it's write reset procedure to vsync
#define VSYNC_PORT		PIOC
#define VSYNC_PIN		PIO_PC13
#define	VSYNC			(REG_PIOC_PDSR & VSYNC_PIN)

//////////////[Enumerations]////////////////////////////////////////////////////////////////////////
// Test patterns
typedef enum CameraTestPatterns
{
	CAM_PATTERN_NONE,
	CAM_PATTERN_SHIFT,
	CAM_PATTERN_BAR,
	CAM_PATTERN_FADE
} CameraTestPatterns;
	
//////////////[Functions]///////////////////////////////////////////////////////////////////////////
void camInit(void);
uint8_t camSetup(void);

void camHardReset(void);
void camRegisterReset(void);
bool camValidID(void);

/*
* Function:
* uint8_t camUpdateWindowSize(void)
*
* Returns the dimensions of the capture window within the camera. This is used to format the data
* retrieved from the buffer correctly.
*
* Inputs:
* Pointers to 16 bit uints where the retrieved window dimensions will be stored
*
* Returns:
* 0 on exit
*
*/
uint8_t camUpdateWindowSize(void);

/*
* Function:
* uint8_t camSetWindowSize(uint16_t hStart, uint16_t hStop, uint16_t vStart, uint16_t vStop)
*
* Sets the size of the image window.
*
* Inputs:
* uint16_t hStart:
*	The x axis pixel location where the camera should start a line
* uint16_t hStop:
*	The x axis pixel loaction where a line should end
* uint16_t vStart:
*	The vertical line that an image should begin from
* uint16_t the vertical line that an image should end at.
*
* Returns:
* 0 on exit
*
*/
uint8_t camSetWindowSize(uint16_t hStart, uint16_t hStop, uint16_t vStart, uint16_t vStop);

void camChangeFormat(uint8_t type);
void camTestPattern(CameraTestPatterns type);

void camRead(void);

#endif /* CAMERA_INTERFACE_H_ */