/*
* motion_functions.h
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 13/08/2017 12:42:12 AM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Defines and function prototypes for motion_function.c which provides high level movement functions
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void funcName(void)
*
*/

#ifndef MOTION_FUNCTIONS_H_
#define MOTION_FUNCTIONS_H_

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_defines.h"
#include <stdlib.h>

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
//PID constants for rotateToHeading
#define RTH_KP	4.0

//PID constants for trackLight
#define TL_KP	10.0
#define TL_KI	0.001

//PID constants for trackLightprox
#define TL_KP	10.0
#define TL_KI	0.001

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* float rotateToHeading(float heading, struct Position *imuData)
*
* Will rotate the robot to face the given heading
*
* Inputs:
* float heading:
*   The heading in degrees that we wish the robot to face (-180 < heading < 180)
* struct Position *imuData:
*   A pointer to the robotPosition structure so we can get imuYaw
*
* Returns:
* Will return 0 if the robot has settled at the desired heading, otherwise will return the signed
* error
*
*/
float rotateToHeading(float heading, struct Position *imuData);

/*
* Function:
* float trackLight(struct Position *imuData)
*
* Robot while attempt to aim itself at a light source
*
* Inputs:
* struct Position *imuData:
*   A pointer to the robotPosition structure
*
* Returns:
* 0 if equilibrium is reached, otherwise will return the proportional error value
*
*/
float trackLight(struct Position *imuData);

float trackLightProx(struct Position *imuData);

/*
* Function:
* char randomMovementGenerator(void)
*
* Will make the robot move around psuedo-randomly
*
* Inputs:
* No Inputs
*
* Returns:
* No return values
*
*/
char randomMovementGenerator(void);

#endif /* MOTION_FUNCTIONS_H_ */