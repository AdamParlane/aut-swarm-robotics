/*
* comm_functions.h
*
* Author : Adam Parlane/Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 15/09/2017 11:17:47 PM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Header file for comm_functions.c
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void commGetNew()
* void commInterpretSwarmMessage(struct MessageInfo message, RobotGlobalStructure *sys)
*
*/

#ifndef COMM_FUNCTIONS_H_
#define COMM_FUNCTIONS_H_

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void commGetNew(RobotGlobalStructure *sys)
*
* Checks for new communications and handlles the interpretation of them
*
* Inputs:
* pointer to frame_info struct and pointer to message_info struct
*
* Returns:
* none
*
*/
void commGetNew(RobotGlobalStructure *sys);

/*
* Function:
* void commInterpretSwarmMessage(RobotGlobalStructure *sys)
*
* Interprets and acts on a received swarm messages
*
* Inputs:
* struct MessageInfo message:
*   TODO: Adam input description
*
* Returns:
* none
*
*/
void commInterpretSwarmMessage(RobotGlobalStructure *sys);

#endif /* COMM_FUNCTIONS_H_ */