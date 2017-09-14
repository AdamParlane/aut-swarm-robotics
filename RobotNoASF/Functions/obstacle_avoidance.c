/*
* obstacle_avoidance.c
*
* Author : Adam Parlane
* Created: 8/08/2017 3:08:20 PM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Functions that perform obstacle avoidance
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* uint8_t scanProximity(void)
* void dodgeObstacle(RobotGlobalStructure *sys)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"
#include "obstacle_avoidance.h"
#include "../Interfaces/twimux_interface.h"
#include "../Interfaces/motor_driver.h"
#include "../Interfaces/prox_sens_interface.h"

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
* Implementation:
* Loops through proximity sensors using a for loop
* CW starting at A, F, E, D, C, B
* Reads each proximity sensor and fills the proximity array with their values
*
*/
void scanProximity(void)
{
	uint8_t index = 0;
	for(uint16_t i = MUX_PROXSENS_A; i <= MUX_PROXSENS_B; i++)
	{
		proximity[index] = proxSensRead(i);
		index++;
	}		
}


/*
* Function:
* void dodgeObstacle(RobotGlobalStructure *sys)
* Will make the robot avoid obstacles while (hopefully remain on the current track
*
* Inputs:
* Aim, the direction robot was heading irrespective of obstacles
* Speed, the speed the robot was heading irrespective of obstacles
*
* Returns:
* No return values
*
* Implementation:
* [WIP] - AP
*
* Improvements:
* [WIP]
*
*/
void dodgeObstacle(RobotGlobalStructure *sys)
{
	scanProximity();	//update proximity readings
	uint16_t proxRange = 0;
	uint8_t indexLeft, indexRight;
	for(uint8_t index = 0; index < 6 ; index++)
	{
		proxRange = index * 60; //convert angle to degrees
		indexLeft = index + 1;
		indexRight = index - 1;
		//keep left and right indexes in range
		if(indexLeft > 5)
			indexLeft = 0;
		if(indexRight > 5)
			indexRight = 5;
		if((sys->pos.targetHeading > (proxRange - 30) && 
		(sys->pos.targetHeading < (proxRange + 30))) || ((index == 0) && 
		(sys->pos.targetHeading > 330) && (sys->pos.targetHeading < 30)))
		{
			if((proximity[index] > OBSTACLE_THRESHOLD) || (proximity[indexLeft] > 1000) || (proximity[indexRight] > 1000))
			{
				if(proximity[indexLeft] > proximity[indexRight])
				{
					sys->pos.targetHeading +=90;
					moveRobot(sys->pos.targetHeading, sys->pos.targetSpeed);//moveLeft
					
				}
				else if (proximity[indexLeft] < proximity[indexRight])
				{
					sys->pos.targetHeading -= 90;
					moveRobot(sys->pos.targetHeading, sys->pos.targetSpeed);//move right
				}
			}
			else 
				moveRobot(sys->pos.targetHeading, sys->pos.targetSpeed);
		}	
	}		
}
