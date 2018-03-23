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
*
*/ 

#ifndef CAMERA_BUFFER_INTERFACE_H_
#define CAMERA_BUFFER_INTERFACE_H_

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////

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
*/
uint8_t camBufferReadData(uint32_t startAddr, uint32_t endAddr, uint8_t *data);

#endif /* CAMERA_BUFFER_INTERFACE_H_ */