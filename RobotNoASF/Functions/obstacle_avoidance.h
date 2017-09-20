/*
* obstacle_avoidance.h
*
* Author : Adam Parlane
* Created: 8/08/2017 3:07:52 PM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Function prototypes for obstacle avoidance system
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void scanProximity(void)
* void dodgeObstacle(RobotGlobalStructure *sys)
*
*/

#ifndef OBSTACLE_AVOIDANCE_H_
#define OBSTACLE_AVOIDANCE_H_

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
#define OBSTACLE_THRESHOLD 0x0100 //Subject to change on testing but seems reasonable
//////////////[Global Variables]///////////////////////////////////////////////////////////////////////////
uint16_t proximity[6];

//////////////[Functions]///////////////////////////////////////////////////////////////////////////

/*
* Function:
* char void scanProximity(void)
*
* Updates latest proximity values in an array
*
* Inputs:
* No Inputs
*
* Returns:
* No return values
*
*/
void scanProximity(RobotGlobalStructure *sys);

/*
* Function:
* void dodgeObstacle(RobotGlobalStructure *sys)
*
* Will make the robot avoid obstacles while (hopefully remain on the current track
*
* Inputs:
* Aim, the direction robot was heading irrespective of obstacles
* Speed, the speed the robot was heading irrespective of obstacles
*
* Returns:
* No return values
*
*
*/
uint8_t dodgeObstacle(RobotGlobalStructure *sys);

void checkForObstacles(RobotGlobalStructure *sys);

uint8_t checkProximity(void);

#define LEFT	0
#define RIGHT	1




#endif /* OBSTACLE_AVOIDANCE_H_ */