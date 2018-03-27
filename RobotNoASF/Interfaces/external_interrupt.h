/*
* external_interrupt.h
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 9/08/2017 9:26:57 AM
*
* Project Repository: https://github.com/wittsend/aut-swarm-robotics
*
* Function prototypes for setting up external interrupts
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* See page 446 for information on PIO interrupts (Section 27)
*
* Functions:
* void extIntInit(void)
* uint8_t extCamWriteToBuffer(void);
*
*/
#ifndef EXTERNAL_INTERRUPT_H_
#define EXTERNAL_INTERRUPT_H_


//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void extIntInit(void)
*
* Initialisation for external interrupts goes in here
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void extIntInit(void);

/*
* Function:
* uint8_t extCamWriteToBuffer(void)
*
* If there is no new data in the buffer, will start the VSYNC ext interrupt to load a frame into
* the camera's FIFO buffer.
*
* Inputs:
* none
*
* Returns:
* Returns 0 if there is new data to be read from the FIFO, otherwise will return 1 while the
* write operation is being performed.
*
*/
uint8_t extCamWriteToBuffer(void);

#endif /* EXTERNAL_INTERRUPT_H_ */