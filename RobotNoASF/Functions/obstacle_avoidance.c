/*
* obstacle_avoidance.c
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
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
* uint8_t scanProxSensors(uint8_t obstacleDetected)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "obstacle_avoidance.h"

//////////////[Functions]///////////////////////////////////////////////////////////////////////////

char  currentDirection = 0;
#define speed 40

void decision(void)
{
	
	//static uint16_t proximity[6], previousProximity[6];
	static uint16_t previousProximity[6];
	memcpy(previousProximity, proximity, 6);
	scanProximity();//proximity);
	signed int desiredDirectionPlus = currentDirection;
	signed int desiredDirectionMinus = currentDirection;
	for(uint8_t index = 0; index < 6; index++)
	{
		if((proximity[index] > OBSTACLE_THRESHOLD) && (((index*60) - currentDirection) > 30))//obstacle detected in the way
		{
			if(proximity[index] > previousProximity[index])//obstacle approaching
			{
				//bash it out, could end up in a function
				//desiredDirection[index] = (index * 60 - 180);
				desiredDirectionPlus = (desiredDirectionPlus + (60*index));
				desiredDirectionMinus = (desiredDirectionMinus - (60*index));				
			}
		}
	}
	//average desired directions
	//choose closest
	if((desiredDirectionPlus - currentDirection) > (desiredDirectionMinus + currentDirection))
	{
		moveRobot(desiredDirectionMinus, speed);
		currentDirection = desiredDirectionMinus;
	}
	else if((desiredDirectionPlus - currentDirection) < (desiredDirectionMinus + currentDirection))
	{
		moveRobot(desiredDirectionPlus, speed);
		currentDirection = desiredDirectionPlus;
	}

}

//CW starting at A, F, E, D, C, B
void scanProximity(void)//uint16_t proximity[6])
{
	uint8_t index = 0;
	for(uint16_t i = MUX_PROXSENS_A; i <= MUX_PROXSENS_B; i++)
	{
		proximity[index] = proxSensRead(i);
		index++;
		
		uint16_t adam = proxSensRead(MUX_PROXSENS_A);
		uint16_t matt = adam;	
	}		
}

signed int dodgeObstacle(signed int aim)
{
	//static uint16_t previousProximity[6];
	//memcpy(previousProximity, proximity, 6);
	scanProximity();//proximity);

	if( (aim > 330) || (aim < 30))//A
	{
		if((proximity[0] > OBSTACLE_THRESHOLD) || (proximity[1] > 1000) || (proximity[5] > 1000))//Prox A obstacle
		{
			if(proximity[1] > proximity[5])
			{
				aim -= 60;
				moveRobot(aim, 40);
			}
			else if (proximity[1] < proximity[5])
			{
				aim += 60;
				moveRobot(aim, 40);//move left
			}
		}
	}
	if( (aim > 30) && (aim < 90))//F
	{
		if((proximity[1] > OBSTACLE_THRESHOLD) || (proximity[2] > 1000) || (proximity[0] > 1000) )//Prox F obstacle
		{
			if(proximity[2] > proximity[0])
			{
				aim -= 60;
				moveRobot(aim, 40);
			}
			else if (proximity[2] < proximity[0])
			{
				aim += 60;
				moveRobot(aim, 40);//move left
			}
		}
	}
	if( (aim > 90) && (aim < 150))//E
	{
		if((proximity[2] > OBSTACLE_THRESHOLD) || (proximity[3] > 1000) || (proximity[1] > 1000) )//Prox E obstacle
		{
			if(proximity[3] > proximity[1])
			{
				aim -= 60;
				moveRobot(aim, 40);
			}
			else if (proximity[3] < proximity[1])
			{
				aim += 60;
				moveRobot(aim, 40);//move left
			}
		}
	}
	if( (aim > 150) && (aim < 210))//D
	{
		if((proximity[3] > OBSTACLE_THRESHOLD) || (proximity[4] > 1000) || (proximity[2] > 1000) )//Prox D obstacle
		{
			if(proximity[4] > proximity[2])
			{
				aim -= 60;
				moveRobot(aim, 40);
			}
			else if (proximity[4] < proximity[2])
			{
				aim += 60;
				moveRobot(aim, 40);//move left
			}
		}
	}
	if( (aim > 210) && (aim < 270))//C
	{
		if((proximity[4] > OBSTACLE_THRESHOLD) || (proximity[5] > 1000) || (proximity[3] > 1000) )//Prox C obstacle
		{
			if(proximity[5] > proximity[3])
			{
				aim -= 60;
				moveRobot(aim, 40);
			}
			else if (proximity[5] < proximity[3])
			{
				aim += 60;
				moveRobot(aim, 40);//move left
			}
		}
	}
	if( (aim > 270) && (aim < 360))//B
	{
		if((proximity[5] > OBSTACLE_THRESHOLD) || (proximity[0] > 1000) || (proximity[4] > 1000) )//Prox B obstacle
		{
			if(proximity[0] > proximity[4])
			{
				aim -= 60;
				moveRobot(aim, 40);
			}
			else if (proximity[0] < proximity[4])
			{
				aim += 60;
				moveRobot(aim, 40);//move left
			}
		}
	}
	if(aim > 360)
		aim -= 360;	
	if(aim < 0)
		aim += 360;			
	return aim;		
}
