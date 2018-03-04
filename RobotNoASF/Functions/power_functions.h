/*
* power_functions.h
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 13/09/2017 6:32:24 PM
*
* Project Repository: https://github.com/wittsend/aut-swarm-robotics
*
* Header for the power functions
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void pfPollPower(RobotGlobalStructure *sys)
* uint8_t pfChargeCycleHandler(RobotGlobalStructure *sys)
*
*/

#ifndef POWER_FUNCTIONS_H_
#define POWER_FUNCTIONS_H_

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
//#include "../robot_setup.h"

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void pfPollPower(RobotGlobalStructure *sys)
*
* Will poll data from the battery and fast charge chip if the corresponding flags are set in sys
*
* Inputs:
* RobotGlobalStructure *sys:
*   Pointer to the global robot data structure
*
* Returns:
* none
*
*/
void pfPollPower(RobotGlobalStructure *sys);

/*
* Function:
* uint8_t pfChargeCycleHandler(RobotGlobalStructure *sys)
*
* State machine that handles the charging cycle of the battery
*
* Inputs:
* struct SystemStatesGroup *state
*   Pointer to the sys.states data structure
* RobotGlobalStructure *sys:
*   Pointer to the sys->pos. data structure
*
* Returns:
* Returns 0 when finished charging successfully. Returns 0xFF when a fault has occurred, otherwise
* returns the number of the current state.
*
*/
uint8_t pfChargeCycleHandler(RobotGlobalStructure *sys);

#endif /* POWER_FUNCTIONS_H_ */