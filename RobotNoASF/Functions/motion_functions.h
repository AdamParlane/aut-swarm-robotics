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
* float mfRotateToHeading(float heading, RobotGlobalStructure *sys)
* float mfMoveToHeading(float heading, uint8_t speed, RobotGlobalStructure *sys)
* float mfMoveToHeadingByDistance(float heading, uint8_t speed, uint32_t distance,
*                                  RobotGlobalStructure *sys)
* float mfTrackLight(RobotGlobalStructure *sys)
* float mfTrackLightProx(RobotGlobalStructure *sys)
* char mfRandomMovementGenerator(void) 
*
*/

#ifndef MOTION_FUNCTIONS_H_
#define MOTION_FUNCTIONS_H_

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
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
#define TLP_KP	10.0
#define TLP_KI	0.001

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* float mfRotateToHeading(float heading, RobotGlobalStructure *sys)
*
* Will rotate the robot to face the given heading
*
* Inputs:
* float heading:
*   The heading in degrees that we wish the robot to face (-180 < heading < 180)
* RobotGlobalStructure *sys:
*   A pointer to the sys->pos. structure so we can get IMU.yaw
*
* Returns:
* Will return 0 if the robot has settled at the desired heading, otherwise will return the signed
* error
*
*/
float mfRotateToHeading(float heading, RobotGlobalStructure *sys);

/*
* Function:
* float mfMoveToHeading(float heading, uint8_t speed, RobotGlobalStructure *sys)
*
* Will rotate and then move the robot along the given heading at the given speed.
*
* Inputs:
* float heading:
*   The heading in degrees that we wish the robot to face (-180 < heading < 180)
* uint8_t speed:
*   Absolute speed as a percentage of maximum speed (0-100)
* RobotGlobalStructure *sys:
*   A pointer to the sys->pos. structure so we can get IMU.yaw
*
* Returns:
* Will return 0 if the robot moving along the desired heading, otherwise will return the signed
* error
*
*/
float mfMoveToHeading(float heading, uint8_t speed, RobotGlobalStructure *sys);

/*
* Function:
* float mfMoveToHeadingByDistance(float heading, uint8_t speed, uint32_t distance,
*									 RobotGlobalStructure *sys);
*
* Will allow robot to move along the given heading a given distance.
*
* Inputs:
* float heading:
*   Heading to move along (-180 to 180 degrees)
* uint8_t speed:
*   Percentage of max speed to move at (0-100%)
* uint32_t distance:
*   Distance to travel before stopping.
* struct SystemStates *state
*   Pointer to the sys.states data structure
* RobotGlobalStructure *sys:
*   Pointer to the sys->pos. global structure.
*
* Returns:
* 0 when maneuver is complete, otherwise returns distance remaining before maneuver complete.
*
*/
float mfMoveToHeadingByDistance(float heading, uint8_t speed, uint32_t distance, 
								 RobotGlobalStructure *sys);

/*
* Function:
* float mfTrackLight(RobotGlobalStructure *sys)
*
* Robot while attempt to aim itself at a light source
*
* Inputs:
* RobotGlobalStructure *sys:
*   A pointer to the sys->pos. structure
*
* Returns:
* 0 if equilibrium is reached, otherwise will return the proportional error value
*
*/
float mfTrackLight(RobotGlobalStructure *sys);

/*
* Function:
* float mfTrackLightProx(RobotGlobalStructure *sys)
*
* Function to track a light source using the proximity sensors.
*
* Inputs:
* RobotGlobalStructure *sys:
*   Pointer to the global sys->pos. data structure.
*
* Returns:
* 0 if facing light source, otherwise will return heading error value
*
*/
float mfTrackLightProx(RobotGlobalStructure *sys);

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

void mfStopRobot(RobotGlobalStructure *sys);

#endif /* MOTION_FUNCTIONS_H_ */