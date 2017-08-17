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
/*
* Function:
* uint8_t scanProxSensors(uint8_t obstacleDetected)
*
* Scans all proximity sensors. If any sensor has a value below the distance threshold then it
* returns a 1. Tells the program when it should switch to the obstacle avoidance state.
*
* Inputs:
* uint8_t obstacleDetected
*   A reference parameter that returns a 1 if anything is detected by the proximity sensors, 
*   otherwise returns a 0.
*
* Returns:
* Returns non zero on error when reading proximity sensors. 
* TODO: Define error codes for scanProxSensors()
*
* Implementation:
* Reads the value from each proximity sensor one by one using a for loop. If a proximity sensor is
* found to have a value that exceeds the 'in range' threshold (ie something is within 100mm of the
* robot) then set obstacleDetected to 1 and return from the function without error.
*
* Improvements:
* Add error code returns. (requires error returns for proxSensRead())
*
*/
uint8_t scanProxSensors(uint8_t *obstacleDetected)
{
	uint16_t sensorValue = 0;
	*obstacleDetected = 0;  
	for(uint16_t sensor = MUX_PROXSENS_A; sensor <= MUX_PROXSENS_B; sensor++)
	{
		sensorValue = proxSensRead(sensor);
		if (sensorValue > PS_IN_RANGE)
		{
			*obstacleDetected = 1;
			return 0;
		}
	} 
	return 0;
}


typedef struct  
{
	float occupied; //P(occupied | s)
	float empty;	//P(Empty | s)
} P;

void formOccupancyGrid (void)
{
	//this is way too complicated for the micro
	P occupancyGrid[10][10]; //each square is 2 x 2cm
	//Region 1
	uint16_t proximity[6];
	//P occupied = ( ((R-r)/R) + ((B - a)/B) /2) * Max
	//R is max range of prox sens
	//B is max angular displacement
	//r is distance to cell
	//a is angle to axis of cell
	//Max is correction constant
	scanProximity(&proximity[6]);
	for(char a = 0; a < 6; a++)
	{
		for(char i = 0; i < 10; i ++)
		{
			for(char j = 0; j < 10; j++)
			{
				//r = sqrt (i^2 + j^2)
				//if r < proximity[a] * some scaling factor
					//region I
				//else region 2
				occupancyGrid[i][j].occupied = 0.5 * (((P_MAX_DIST - i) / P_MAX_DIST) + ((P_MAX_ANG - angle)/P_MAX_ANG)) / 2;
			}
		}	
	}

}

void scanProximity(uint16_t *proximity[6])
{
	proximity[0] -> proxSensRead(MUX_PROXSENS_A);
	proximity[1] -> proxSensRead(MUX_PROXSENS_B);
	proximity[2] -> proxSensRead(MUX_PROXSENS_C);
	proximity[3] -> proxSensRead(MUX_PROXSENS_D);
	proximity[4] -> proxSensRead(MUX_PROXSENS_E);
	proximity[5] -> proxSensRead(MUX_PROXSENS_F);				
}