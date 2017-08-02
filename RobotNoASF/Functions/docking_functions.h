/*
* docking_functions.h
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 25/07/2017 8:08:11 PM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* [WIP] Contains functions for docking... [WIP]
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void dockRobot(void)
* void updateLineSensorStates(void)
* int8_t getLineDirection(void)
* void followLine(void)
*
*/

#ifndef DOCKING_FUNCTIONS_H_
#define DOCKING_FUNCTIONS_H_

///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "../robot_defines.h"

///////////////Defines//////////////////////////////////////////////////////////////////////////////


///////////////Type Definitions/////////////////////////////////////////////////////////////////////
struct LineSensorArray
//Will store states of the line sensors for use by functions in this modules. This is necessary
//because there is a gray area when the sensor is half on and half off the line, so by establishing
//hysteresis and only changing the stored states when an upper and lower threshold is crossed,
//jitter should be reduced.
{
	uint8_t outerLeft;	
	uint8_t innerLeft;
	uint8_t innerRight;
	uint8_t outerRight;
};



///////////////Functions////////////////////////////////////////////////////////////////////////////
/*
* Function:
* void dockRobot(void)
*
* Function to guide the robot to the dock.
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void dockRobot(void);

/*
* Function:
* void updateLineSensorStates(void)
*
* Sees if any sensors have made a definite state change and loads the states into the line sensor
* state structure for use by other functions in this module.
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void updateLineSensorStates(void);

/*
* Function:
* int8_t getLineDirection(void)
*
* This function examines the states of the line follower sensors and determines the direction and
* urgency factor by which the robot should move to find its way to the centre of the line.
*
* Inputs:
* none
*
* Returns:
* returns a signed integer between -3 and 3 that determines the direction and speed magnitude that
* the robot should move to find the centre of the line.
* A negative output means that the robot should move left to find the line and a positive output
* means that the robot should move right. 0 means keep going straight because no direction data is
* able to be derived from sensor array.
*
*/
int8_t getLineDirection(void);

/*
* Function:
* void followLine(void)
*
* A basic function to follow a line that seems to work ok
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void followLine(void);

#endif /* DOCKING_FUNCTIONS_H_ */