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
#include "../robot_setup.h"
#include <stdlib.h>

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
//PID constants for mfRotateToHeading
#define RTH_KP	4.0

//PID constants for mfMoveToHeading
#define MTH_KP	4.0

//PID constants for mfTrackLight
#define TL_KP	10.0
#define TL_KI	0.001

//PID constants for mfTrackLightprox
#define TL_KP	10.0
#define TL_KI	0.001

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* float mfRotateToHeading(float heading, struct Position *imuData)
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
float mfRotateToHeading(float heading, struct Position *imuData);

/*
* Function:
* float mfMoveToHeading(float heading, uint8_t speed, struct Position *imuData)
*
* Will rotate and then move the robot along the given heading at the given speed.
*
* Inputs:
* float heading:
*   The heading in degrees that we wish the robot to face (-180 < heading < 180)
* uint8_t speed:
*   Absolute speed as a percentage of maximum speed (0-100)
* struct Position *imuData:
*   A pointer to the robotPosition structure so we can get imuYaw
*
* Returns:
* Will return 0 if the robot moving along the desired heading, otherwise will return the signed
* error
*
*/
float mfMoveToHeading(float heading, uint8_t speed, struct Position *imuData);

float moveForwardByDistance(uint16_t distance, struct Position *posData);

/*
* Function:
* float mfTrackLight(struct Position *imuData)
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
float mfTrackLight(struct Position *imuData);

float mfTrackLightProx(struct Position *imuData);

/*
* Function:
* char mfRandomMovementGenerator(void)
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
char mfRandomMovementGenerator(void);

#endif /* MOTION_FUNCTIONS_H_ */