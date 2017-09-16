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

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
#define MC_STRAIGHT 0xD1
#define MC_STOP		0xD0
#define MC_CW		0xD2
#define MC_CCW		0xD3

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
void manualControl(RobotGlobalStructure *sys);


#endif /* MANUAL_MODE_H_ */