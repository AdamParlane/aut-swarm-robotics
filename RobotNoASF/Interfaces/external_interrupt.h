/*
* external_interrupt.h
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 9/08/2017 9:26:57 AM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Function prototypes for setting up external interrupts
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* See page 446 for information on PIO interrupts (Section 27)
*
* Functions:
* void funcName(void)
*
*/
#ifndef EXTERNAL_INTERRUPT_H_
#define EXTERNAL_INTERRUPT_H_

///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "../robot_defines.h"

///////////////Functions////////////////////////////////////////////////////////////////////////////
void extIntInit(void);
void imuReadFifo(void);
#endif /* EXTERNAL_INTERRUPT_H_ */