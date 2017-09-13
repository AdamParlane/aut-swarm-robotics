/*
* charging_functions.h
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 13/09/2017 6:32:24 PM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Header for the charging functions
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* uint8_t cfChargeCycleHandler(struct Position *imuData)
*
*/

#ifndef CHARGING_FUNCTIONS_H_
#define CHARGING_FUNCTIONS_H_

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* uint8_t cfChargeCycleHandler(struct Position *imuData)
*
* State machine that handles the charging cycle of the battery
*
* Inputs:
* struct Position *imuData:
* Pointer to the robotPosition data structure
*
* Returns:
* Returns 0 when finished charging successfully. Returns 0xFF when a fault has occurred, otherwise
* returns the number of the current state.
*
*/
uint8_t cfChargeCycleHandler(struct Position *imuData);

#endif /* CHARGING_FUNCTIONS_H_ */