/*
* obstacle_avoidance.h
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 8/08/2017 3:07:52 PM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Function protoypes for obstacle avoidance system
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
*
*/

#ifndef OBSTACLE_AVOIDANCE_H_
#define OBSTACLE_AVOIDANCE_H_
//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
#define OBSTACLE_THRESHOLD 0x1000

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
void decision(void);
void scanProximity(uint16_t *proximity[6]);


#endif /* OBSTACLE_AVOIDANCE_H_ */