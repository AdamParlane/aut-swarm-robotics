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
* uint8_t camBufferWriteFrame(void);
* uint8_t camBufferReadWin(uint32_t left, uint32_t top, uint32_t width, uint32_t height,
*								uint16_t dataBuffer[], uint32_t bufferSize);
* uint8_t camBufferReadWin2(uint32_t hStart, uint32_t hStop, uint32_t vStart, uint32_t vStop,
*								uint16_t dataBuffer[], uint32_t bufferSize)
*
*/ 

#ifndef CAMERA_BUFFER_INTERFACE_H_
#define CAMERA_BUFFER_INTERFACE_H_

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
// Writing to buffer from camera. These macros are defined for public use so that the external
// interrupt module can use them. Saves on unecessary function calls from within the interrupt
// service routines.

// Buffer pin			SAM4 port/pin	Function			Type		Robot Pin Name
#define WE_PORT			PIOC
#define WE_PIN			PIO_PC7			//Write Enable		Output		VB_WE
#define WRST_PORT		PIOA
#define WRST_PIN		PIO_PA24		//Write Reset		Output		VB_WRST

//Image constants:
//Physical width and height of the image as it is stored in the buffer. No software windowing
//operation will be able to go beyond these limits as it would cause the read pointer to overflow
//on the buffer
#define CAM_IMAGE_BPP		2		//Bytes per pixel
#define CAM_IMAGE_WIDTH		311
#define CAM_IMAGE_HEIGHT	249		//393,216 bytes/2/311. The actual picture is much less than this
									//but this specifies the absolute maximum height that could be
									//read from the buffer. This might get smaller once I know how
									//tall the picture actually is.

//The margins define how far in from the edge that you will obtain "clean" image data:
#define CAM_TOP_MARGIN		9
#define CAM_BOTTOM_MARGIN	0
#define CAM_LEFT_MARGIN		9		
#define CAM_RIGHT_MARGIN	1

//Macros
#define camBufferWriteDisable	WE_PORT->PIO_CODR	|= WE_PIN	//Buffer write disable (Active high 
																//via NAND gate)
#define camBufferWriteEnable	WE_PORT->PIO_SODR	|= WE_PIN	//Buffer write enable
#define camBufferWriteResetOn	WRST_PORT->PIO_CODR	|= WRST_PIN	//Buffer write reset
#define camBufferWriteResetOff	WRST_PORT->PIO_SODR |= WRST_PIN	//Buffer write on (not in reset)

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void camBufferInit()
*
* Initialises the micro hardware required to communicate with the camera buffer
*
* Inputs:
* none
*
* Returns:
* 0 on success, otherwise non zero
*
*/
uint8_t camBufferInit(void);

/*
* Function:
* void camBufferWriteStop(void)
*
* Disables writing to the FIFO
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void camBufferWriteStop(void);
/*
* Function:
* void camBufferWriteStart(void)
*
* Enables writing to the FIFO
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void camBufferWriteStart(void);
/*
* Function:
* void camBufferWriteReset(void)
*
* Resets the write address pointer to 0
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void camBufferWriteReset(void);

/*
* Function:
* void camBufferReadStop(void)
*
* Disables reading from the FIFO
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void camBufferReadStop(void);
/*
* Function:
* void camBufferReadStart(void)
*
* Enables reading from the FIFO
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void camBufferReadStart(void);
/*
* Function:
* void camBufferReadReset(void)
*
* Resets the read address pointer to 0
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void camBufferReadReset(void);

/*
* Function:
* uint8_t camBufferReadSequence(uint32_t startAddr, uint32_t endAddr, uint16_t *data)
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
*/
uint8_t camBufferReadSequence(uint32_t startAddr, uint32_t endAddr, uint16_t *data);

/*
* Function:
* uint8_t camBufferReadWin(uint32_t left, uint32_t top, uint32_t width, uint32_t height,
*								uint16_t dataBuffer[], uint32_t bufferSize)
*
* This function will read a small portion of the image stored in the camera buffer
*
* Inputs:
* uint32_t left:
*	The left (x) coordinate of the image to be retrieved
* uint32_t top:
*	The top (y) coordinate of the image to be retrieved
* uint32_t width:
*	The width (in pixels) of the image to be retrieved
* uint32_t height:
*	The height (in pixels) of the image to be retrieved
* uint16_t dataBuffer[]:
*	Pointer to the array where the image will be stored
* uint32_t bufferSize:
*	Length of the storage array in elements. Used for error checking
*
* Returns:
* 1 if the coordinates and dimensions given are out of range of the image stored in the buffer, or
* if the length of the buffer is not big enough to store the retrieved data.
*
*/
uint8_t camBufferReadWin(uint32_t left, uint32_t top, uint32_t width, uint32_t height,
							uint16_t dataBuffer[], uint32_t bufferSize);

/*
* Function:
* uint8_t camBufferReadWin2(uint32_t hStart, uint32_t hStop, uint32_t vStart, uint32_t vStop,
*								uint16_t dataBuffer[], uint32_t bufferSize)
*
* This function will read a small portion of the image stored in the camera buffer
*
* Inputs:
* uint32_t hStart:
*	Horizontal start position
* uint32_t hStop:
*	Horizontal stop position
* uint32_t vStart:
*	Vertical start position
* uint32_t vStop:
*	Vertical stop position
* uint16_t dataBuffer[]:
*	Pointer to the array where the image will be stored
* uint32_t bufferSize:
*	Length of the storage array in elements. Used for error checking
*
* Returns:
* 1 if the coordinates and dimensions given are out of range of the image stored in the buffer, or
* if the length of the buffer is not big enough to store the retrieved data.
*
*/
uint8_t camBufferReadWin2(uint32_t hStart, uint32_t hStop, uint32_t vStart, uint32_t vStop,
							uint16_t dataBuffer[], uint32_t bufferSize);

/*
* Function:
* uint8_t camBufferWriteFrame(void)
*
* Instructs the camera to write one frame to the FIFO. The process is handled by an external
* interrupt.
*
* Inputs:
* none
*
* Returns:
* Returns 0 when there is a frame ready to be read from the buffer
*
*/
uint8_t camBufferWriteFrame(void);

#endif /* CAMERA_BUFFER_INTERFACE_H_ */