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



void decision(void)
{
	static uint16_t proximity[6], previousProximity[6];
	memcpy(previousProximity, proximity, 6);
	scanProximity(&proximity);
	for(char index = 0; index < 6; index++)
	{
		if(proximity[index] > OBSTACLE_THRESHOLD)//obstacle detected
		{
			if(proximity[index] > previousProximity[index])//obstacle approaching
			{
				//bash it out, could end up in a function
				
			}
		}
	}
	
}

//CW starting at A, F, E, D, C, B
void scanProximity(uint16_t *proximity[6])
{
	char index = 0;
	for(char i = MUX_PROXSENS_A; i < MUX_PROXSENS_B; i++)
	{
		proximity[index] = proxSensRead(i);
		index++;
	}			
}