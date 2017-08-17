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
* uint8_t scanProxSensors(uint8_t obstacleDetected)
*
*/

#ifndef OBSTACLE_AVOIDANCE_H_
#define OBSTACLE_AVOIDANCE_H_
//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* uint8_t scanProxSensors(uint8_t obstacleDetected)
*
* Scans all proximity sensors. If any sensor has a value below the distance threshold then it
* returns a 1. Tells the program when it should switch to the obstacle avoidance state.
*
* Inputs:
* uint8_t obstacleDetected
*   A reference parameter that returns a 1 if anything is detected by the proximity sensors,
*   otherwise returns a 0.
*
* Returns:
* Returns non zero on error when reading proximity sensors.
* TODO: Define error codes for scanProxSensors()
*
*/
uint8_t scanProxSensors(uint8_t *obstacleDetected);



#endif /* OBSTACLE_AVOIDANCE_H_ */