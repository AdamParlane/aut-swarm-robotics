/*
* manual_mode.h
*
* Author : Adam Parlane
* Created: 1/8/2017
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Contains the definitions and functions required for manual mode
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
*
* Functions:
* manualControl(message)
*
*/


#ifndef MANUAL_MODE_H_
#define MANUAL_MODE_H_

///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "../robot_defines.h"

///////////////Defines//////////////////////////////////////////////////////////////////////////////
#define MANUAL_STRAIGHT 0xD1
#define MANUAL_STOP		0xD0

///////////////Functions////////////////////////////////////////////////////////////////////////////
void manualControl(struct message_info tmessage);


#endif /* MANUAL_MODE_H_ */