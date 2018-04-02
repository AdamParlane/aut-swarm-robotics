/*
* camera_interface.h
*
* Author : Brae HW (bhw11@hotmail.co.nz), Matthew Witt, Mansel Jeffares
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
* bool camValidID(void)
* void camHardReset(void);
* uint8_t camUpdateWindowSize(void);
* uint8_t camSetWindowSize(uint16_t hStart, uint16_t hStop, uint16_t vStart, uint16_t vStop);
* void camTestPattern(CameraTestPatterns type);
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
#define	VSYNC			(VSYNC_PORT->PIO_PDSR & VSYNC_PIN)

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
/*
* Function:
* uint8_t camInit(void)
*
* Initialises the uC's hardware for working with the camera, and set's up the camera ready for use.
*
* Inputs:
* none
*
* Returns:
* Returns 0 if initialisation was successful, otherwise returns non zero.
*
*/
uint8_t camInit(void);

/*
* Function:
* bool camValidID(void)
*
* Checks camera is responding and has correct ID
*
* Inputs:
* None
*
* Returns:
* 0 If the camera is not responding, or replies with an invalid ID
*
*/
bool camValidID(void);

/*
* Function:
* void camHardReset(void)
*
* Performs a hard reset of the camera device.
*
* Inputs:
* None
*
* Returns:
* None
*
*/
void camHardReset(void);

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

/*
* Function:
* void camTestPattern(CameraTestPatterns type)
*
* Will output a test pattern specified by the parameter
*
* Inputs:
* CameraTestPatterns type:
*	An enum that contains the value of different test patterns that can be displayed
*
* Returns:
* none
*
*/
void camTestPattern(CameraTestPatterns type);

#endif /* CAMERA_INTERFACE_H_ */